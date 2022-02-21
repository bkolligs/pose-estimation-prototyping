#include "ekf.h"

#include <iostream>   //DEBUG

namespace orientation_ekf {

void Timer::calculateDeltaT(void) {
    curTime_ = std::chrono::steady_clock::now();

    /* Calculate the time step and update the member variable */
    const std::chrono::duration<float> timeElapsed = curTime_ - lastTime_;
    deltaT_ = timeElapsed.count();

    /* Update the last time */
    lastTime_ = curTime_;
}

OrientationEKF::OrientationEKF() {   // TODO what should this default
                                     // constructor look like?
    receivedIMUInitData_ =
        true;   // TODO change these to a legit value. Only have these here for
                // ease of unit testing because functions to set these values
                // are not existent/public
    rateIMU_ = 150;   // TODO change these to a legit value. Only have these
                      // here for ease of unit testing because functions to set
                      // these values are not existent/public
}

bool OrientationEKF::is_valid_rotation_matrix(Eigen::Matrix<double, 3, 3> R) {
    return (R * R.transpose()).isApprox(Eigen::Matrix<double, 3, 3>::Identity(3, 3), 1e-5);
}

void OrientationEKF::setnoiseLambdaG(double lambdaG) {
    noiseLambdaG = -lambdaG;
}

void OrientationEKF::setnoiseLambdaA(double lambdaA) {
    noiseLambdaA = -lambdaA;
}

Eigen::Matrix<double, 9, 9> OrientationEKF::getF() {
    return F_;
}

Eigen::Matrix<double, 9, 12> OrientationEKF::getG() {
    return G_;
}

Eigen::Matrix<double, 9, 9> OrientationEKF::getPhi() {
    return Phi_;
}

Eigen::Matrix<double, 9, 9> OrientationEKF::getQdk() {
    return Qdk_;
}

Eigen::Quaternion<double> OrientationEKF::getOriQuat() {
    Eigen::Quaternion<double> output(state_(0), state_(1), state_(2), state_(3));
    return output;
}

Eigen::Matrix<double, 4, 1> OrientationEKF::getOriValues() {
    Eigen::Matrix<double, 4, 1> output = state_.block(0, 0, 4, 1);
    return output;
}

int32 OrientationEKF::computeF(
    const Vector3& specForce,
    const Eigen::Matrix<double, 3, 3>& RBodyToNavNext) {
    if (not is_valid_rotation_matrix(RBodyToNavNext)) {
        return POSE_EKF_MALFORMED_ROT_MATRIX_ERR_EID;
    }
    /* Zero out the F matrix */
    F_ = Eigen::Matrix<double, 9, 9>::Zero();
    /* Store the relevant values in F */
    toSkew(specForce, specForceSkew_);

    /* Why is this a negative one? */
    F_.block<3, 3>(0, 3) = -1 * RBodyToNavNext;
    F_.block<3, 3>(3, 3) =
        Eigen::Matrix<double, 3, 3>::Identity() * noiseLambdaG;
    F_.block<3, 3>(6, 6) =
        Eigen::Matrix<double, 3, 3>::Identity() * noiseLambdaA;
    return CFE_SUCCESS;
}

int32 OrientationEKF::computeG(
    const Eigen::Matrix<double, 3, 3> RBodyToNavNext) {
    if (not is_valid_rotation_matrix(RBodyToNavNext)) {
        return POSE_EKF_MALFORMED_ROT_MATRIX_ERR_EID;
    }
    /* Zero out the G matrix */
    G_ = Eigen::Matrix<double, 9, 12>::Zero();
    G_.block<3, 3>(0, 0) = -1 * RBodyToNavNext;
    G_.block<3, 3>(0, 6) = -1 * RBodyToNavNext;
    G_.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    G_.block<3, 3>(6, 9) = Eigen::Matrix<double, 3, 3>::Identity();
    return CFE_SUCCESS;
}

int32 OrientationEKF::computePhiAndQdk(
    const Vector3& specForce,
    const Eigen::Matrix<double, 3, 3>& RBodyToNavNext) {
    if (not is_valid_rotation_matrix(RBodyToNavNext)) {
        return POSE_EKF_MALFORMED_ROT_MATRIX_ERR_EID;
    }

    /**
     * This piece of code uses the Van Loan Algorithm for computing integrals
     * involving the matrix exponential:
     * https://www.cs.cornell.edu/cv/ResearchPDF/computing.integrals.involving.Matrix.Exp.pdf
     * And inspired by pg.143 (equations 4.113, 4.114) of "AIDED NAVIGATION GPS
     * with High Rate Sensors" by Jay A. Farrell
     */
    /* Formulate larger matrix, and compute F as 9x9 */
    computeF(specForce, RBodyToNavNext);

    /* Compute G_ */
    computeG(RBodyToNavNext);

    /* Empty matrix */

    /* Fill in the A matrix according to the Van Loan Algorithm */
    A_.block<9, 9>(0, 0) = -F_;
    A_.block<9, 9>(9, 9) = F_.transpose();
    A_.block<9, 9>(0, 9) = G_ * (noiseIMU_)*G_.transpose();
    A_ = A_ * deltaT_;

    /* Matrix exponential and extract values for Phi and Qdk*/
    Eigen::Matrix<double, 18, 18> AExp = A_.exp();
    Phi_ = AExp.block<9, 9>(9, 9).transpose();
    Qdk_ = Phi_ * AExp.block<9, 9>(0, 9);
    return CFE_SUCCESS;
}

int32 OrientationEKF::stationaryMeasurementUpdate(
    const Eigen::Matrix<double, 3, 3>& RBodyToNav) {
    if (not is_valid_rotation_matrix(RBodyToNav)) {
        return POSE_EKF_MALFORMED_ROT_MATRIX_ERR_EID;
    }
    isRoverStationary = false;

    /* Stationary update matrix */
    Ha_ = Eigen::Matrix<double, 3, 9>::Zero();

    /* Known gravity */
    Vector3 measGravity(0, 0, gravity_);
    Eigen::Matrix<double, 3, 3> gravSkew(3, 3);
    toSkew(measGravity, gravSkew);
    Ha_.block<3, 3>(0, 0) = -1 * gravSkew;
    Ha_.block<3, 3>(0, 6) = RBodyToNav;

    /* Measurement and noise matrices */
    Vector3 predOutput = Vector3::Zero();
    Vector3 measOutput = Vector3::Zero();

    /* Predicted measurement is gravity */
    predOutput << predGravity_(0), predGravity_(1), predGravity_(2);

    /* Actual measurement is actual gravity */
    measOutput << measGravity(0), measGravity(1), measGravity(2);

    /* Innovation of the gravity measurement */
    Vector3 z = measOutput - predOutput;

    /* Call the filter here */
    measurementUpdate(Ha_.block<2, 9>(0, 0), noiseAcc_.block<2, 2>(0, 0), z.block<2, 1>(0, 0),
        ACCELEROMETER);
    return CFE_SUCCESS;
}

void OrientationEKF::handleSunSensor(const Vector3& measSunSensor) {
    /* Sun sensor update message can be sent here perhaps through the syslog */
    /* Input is a 3D sun ray in the inertial frame */
    Quaternion bodyOrientation(state_(0), state_(1), state_(2), state_(3));
    Quaternion bodyToNavOrientation = bodyOrientation.inverse();

    /* Stationary update matrix Hs is 3x9 */
    Hs_ = Eigen::Matrix<double, 3, 9>::Zero();

    /* Noise matrix */
    Eigen::Matrix<double, 3, 3> Rs(3, 3);
    const double angleUncertainty = 0.1;
    Rs << angleUncertainty / 180.0 * M_PI, 0.0, 0.0, 0.0,
        angleUncertainty / 180 * M_PI, 0.0, 0.0, 0.0,
        angleUncertainty / 180 * M_PI;

    /* Known sensor reading */
    /* Predict the x-axis in place of a true sensor reading */
    Vector3 xAxis{1, 0, 0};
    Vector3 predSun = bodyToNavOrientation * xAxis;
    /* Copy the measured sun sensor data */
    Vector3 measSun = {measSunSensor(0), measSunSensor(1), measSunSensor(2)};
    /* Normalize the model output */
    measSun = measSun / measSun.norm();

    Eigen::Matrix<double, 3, 3> sunSkew(3, 3);
    toSkew(measSun, sunSkew);
    Hs_.block<3, 3>(0, 0) = -1 * sunSkew;

    /* No corrections to pitch and roll */
    Hs_(0, 1) = 0.0;
    Hs_(1, 0) = 0.0;

    Vector3 predOutput = Vector3::Zero();
    Vector3 measOutput = Vector3::Zero();

    /* Predicted heading */
    predOutput << predSun(0), predSun(1), predSun(2);

    /* Measurement from ephemeris */
    measOutput << measSun(0), measSun(1), measSun(2);

    /* Predicted gravity */
    Vector3 z = measSun - predSun;

    /* Now in the EKF only correct heading error */
    measurementUpdate(Hs_.block<2, 9>(0, 0), Rs.block<2, 2>(0, 0), z.block<2, 1>(0, 0),
        SUN_SENSOR);
}

/* This is the measurement update used by Haidar, will need to not use dynamic
 * matrices like this */
void OrientationEKF::measurementUpdate(const Eigen::Matrix<double, 2, 9>& H,
                                       const Eigen::Matrix<double, 2, 2>& R,
                                       const Eigen::Vector2d& z,
                                       const Sensor sensor_type) {
    /* Compute the Kalman gain */
    Eigen::Matrix<double, 9, 2> K =
        covariance_ * H.transpose() *
        ((R + H * covariance_ * H.transpose()).inverse());   // fix auto later

    // compute state correction
    Eigen::Matrix<double, 9, 1> dx = K * z;

    // rotation update
    Vector3 rotationUpdate = dx.block<3, 1>(0, 0);
    Eigen::Matrix<double, 3, 3> P(3, 3);

    // only correct the z-axis (yaw)
    if (sensor_type == SUN_SENSOR) {
        rotationUpdate(0) = 0;
        rotationUpdate(1) = 0;
    }
    toSkew(rotationUpdate, P);

    // predicted orientation
    Quaternion predBodyOrientation(state_(0), state_(1), state_(2), state_(3));
    Eigen::Matrix<double, 3, 3> RNavToBody =
        predBodyOrientation.toRotationMatrix();
    Eigen::Matrix<double, 3, 3> RNavToBodyNext =
        RNavToBody * (Eigen::Matrix<double, 3, 3>::Identity() - P);

    // reorthogonalize matrix (make it into a rotation matrix)... use SVD
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd(
        RNavToBodyNext, Eigen::DecompositionOptions::ComputeFullU |
                            Eigen::DecompositionOptions::ComputeFullV);
    RNavToBodyNext = svd.matrixU() * ((svd.matrixV()).transpose());

    /* Compute the quaternion update */
    Quaternion bodyOrientationNext(RNavToBodyNext);

    state_.block<4, 1>(0, 0) << bodyOrientationNext.w(),
        bodyOrientationNext.x(), bodyOrientationNext.y(),
        bodyOrientationNext.z();

    /* update the acceleration bias and the gyroscope bias */
    state_.block<3, 1>(4, 0) += dx.block<3, 1>(3, 0);
    state_.block<3, 1>(7, 0) += dx.block<3, 1>(6, 0);

    /* Update the covariance */
    covariance_ = (CovarianceMatrix::Identity() - K * H) * covariance_;

    /* Make the matrix symmetric */
    covariance_ = (covariance_ + covariance_.transpose()) / 2.0;
}

int32 OrientationEKF::handleIMU(const Vector3& measAng,
                                const Vector3& measAcc) {
                                        imuCount_++;
    if (imuCount_ < numInitIMUData_) {
        // Initialize the biases by averaging many values at the beginning. 
        initSumAccel_ +=
            measAcc;   // TODO + or -? Haidar's imit_ekf.cpp uses -=
        initSumGyro_ += measAng;
    } else {   // We've received enough IMU data; proceed to initialize the EKF
        if (not isEKFInitialized_) {
            receivedIMUInitData_ = true;
            initializeEKF();
        }
    }

    /* Current body orientation */
    Quaternion prevBodyOrientation{state_(0), state_(1), state_(2), state_(3)};

    /* Current accelerometer bias */
    Vector3 biasAccel{state_(4), state_(5), state_(6)};

    /* Current gyro bias */
    Vector3 biasGyro{state_(7), state_(8), state_(9)};

    /* Subtract out gyroscope bias to obtain correct estimate of gyro */
    Vector3 corrGyro = -1 * (measAng - biasGyro);

    /* Do the same for the accelerometer */
    Vector3 corrAcc = measAcc - biasAccel;

    /* Convert this to differential rotation expressed by a quaternion */
    float deltaGyro = corrGyro.norm() * deltaT_ / 2.0;
    Vector3 xyz = sin(deltaGyro) * measAng / corrGyro.norm();
    Quaternion deltaOri{cos(deltaGyro), xyz(0), xyz(1), xyz(2)};

    /* Update the orientation based off of our estimated orientation error */
    Quaternion newBodyOrientation = deltaOri * prevBodyOrientation;

    /**
     * Get the average quaternion by spherical interpolation
     * Sources:
     * 1. https://en.wikipedia.org/wiki/Slerp
     * 2. https://www.youtube.com/watch?v=ibkT5ao8kGY&t=444s
     * 3. https://www.youtube.com/watch?v=x1aCcyD0hqE
     */
    Quaternion newBodyOriAvg =
        prevBodyOrientation.slerp(0.5, newBodyOrientation);

    /**
     * Body orientation we've been dealing with is the navigation to body frame
     * rotation Now we need body to navigation frame: invert the quaternion. We
     * use this to represent specific force. Haidar mentions section 5.9 of a
     * Principles of GNSS book: Groves, Paul D. Principles of GNSS, Inertial,
     * and Multisensor Integrated Navigation Systems . Second edition. Boston:
     * Artech House, 2013. Print.
     */
    Quaternion newBodyToNavAvg = newBodyOriAvg.inverse();
    /* Rotate specific force into inertial frame */
    Vector3 forceInertial = newBodyToNavAvg * corrAcc;
    /* Get acceleration in inertial frame */
    Vector3 accInertial = forceInertial - trueGravity_;
    /* Store the updated orientation in the state; this is time propagation step
     */
    state_ << newBodyOrientation.w(), newBodyOrientation.x(),
        newBodyOrientation.y(), newBodyOrientation.z(), biasGyro(0),
        biasGyro(1), biasGyro(2), biasAccel(0), biasAccel(1), biasAccel(2);

    /* Update covariance */
    Eigen::Matrix<double, 3, 3> RBodyToNavNext =
        newBodyOrientation.inverse().toRotationMatrix();
    /* Compute the state transition matrix Phi */
    if (computePhiAndQdk(forceInertial, RBodyToNavNext) != CFE_SUCCESS) {
        return POSE_EKF_COMPUTE_PHI_QDK_ERR_EID;
    }
    // update covariance (15x15)
    covariance_ = Phi_ * covariance_ * Phi_.transpose();

    /**
     * Measurement update using the accelerometer to correct roll and pitch
     * If a certain threshold of accelerometer readings are close enough
     * to the gravity vector, robot is stationary.
     */
    if (abs(corrAcc.norm() - gravity_) < accelerationThresh_) {
        accelerationCount_++;
        predGravitySum_ += RBodyToNavNext * corrAcc;
    } else {
        /* Zero out the stationary counts since we're moving */
        accelerationCount_ = 0;
        predGravitySum_ = Vector3::Zero();
        /* FIXME: This flag current not being set to true ever */
        isRoverStationary = false;
    }

    /* If there is a certain amount of stationary measurements, use acceleration
     * data */
    if (accelerationCount_ > stationaryCounts) {
        /* Predict gravity in navigation frame by averaging and store prediction
         */
        predGravity_ = predGravitySum_ / accelerationCount_;
        /* Zero out the counters */
        accelerationCount_ = 0;
        predGravitySum_ = Vector3::Zero();
        /* Perform the measurement update for stationary rover */
        stationaryMeasurementUpdate(RBodyToNavNext);
    }
    return CFE_SUCCESS;
}

int32 OrientationEKF::initializeEKF() {
    /**
     * This function needs to initialize the EKF before using it to predict
     * state.
     * We do this in the following steps.
     * These steps are repeated throughout the function for clarity.
     * 1. Receive a certain number of IMU measurements
     * 2. Use these measurements to average the received linear acceleration and
     * angular velocity
     * 3. Compute initial gyroscope biases
     * 4. Initialize the roll and pitch
     * 5. Initialize the yaw to zero
     * 6. Use the initial Euler angles to create initial orientation of rover
     * and place into state vector
     * 7. Compute timestep using the desired run rate of the IMU
     * 8. Initialize noise terms in the IMU noise matrix, and accelerometer
     * noise matrix
     * 9. Initialize covariance
     */

    Vector3 avgAccel = Vector3::Zero();
    Vector3 avgGyroBias = Vector3::Zero();

    //   1. Receive a certain number of IMU measurements
    if (receivedIMUInitData_) {
        //   2. Use these measurements to average the received linear
        //   acceleration and angular velocity
        avgAccel = initSumAccel_ / numInitIMUData_;
        //   3. Compute initial gyroscope biases
        avgGyroBias = initSumGyro_ / numInitIMUData_;
    } else {
        return POSE_EKF_IMU_INITDATA_ERR_EID;
    }

    //   4. Initialize the roll and pitch
    double initRoll = atan2(-avgAccel(1), -avgAccel(2));
    double initPitch = atan2(avgAccel(0), sqrt(avgAccel(1) * avgAccel(1) +
                                               avgAccel(2) * avgAccel(2)));
    //   5. Initialize the yaw to zero
    double initYaw = 0;

    //   6. Use the initial Euler angles to create initial orientation of rover
    //   and place into state vector
    Quaternion initBodyOri = Eigen::AngleAxisd(initYaw, unitZ) *
                             Eigen::AngleAxisd(initPitch, unitY) *
                             Eigen::AngleAxisd(initRoll, unitX);

    /* Initial state vector */
    state_ << initBodyOri.w(), initBodyOri.x(), initBodyOri.y(),
        initBodyOri.z(), avgGyroBias(0), avgGyroBias(1), avgGyroBias(2), 0.0,
        0.0, 0.0;

    //   7. Compute timestep using the desired run rate of the IMU
    deltaT_ = 1.0 / rateIMU_;

    //   8. Initialize noise terms in the IMU noise matrix, and accelerometer
    //   noise matrix
    /* First the IMU noise matrix */
    for (int i = 0; i < 3; i++) {
        noiseIMU_(i, i) = sigmaAccWhiteNoise_ * sigmaAccWhiteNoise_;
        noiseIMU_(i + 3, i + 3) = sigmaGyro_ * sigmaGyro_;
        noiseIMU_(i + 6, i + 6) = sigmaGyroWhiteNoise_ * sigmaGyroWhiteNoise_;
        noiseIMU_(i + 9, i + 9) = sigmaAcc_ * sigmaAcc_;
    }

    /* Accelerometer noise matrix */
    noiseAcc_ = NoiseAccMatrix::Identity() *
                (sigmaAccWhiteNoise_ * sigmaAccWhiteNoise_);

    //   9. Initialize covariance
    double dataHZ = numInitIMUData_ / rateIMU_;
    double covRollPitch =
        ((sigmaAccWhiteNoise_ / gravity_) * (sigmaAccWhiteNoise_ / gravity_)) /
        dataHZ;
    covariance_.block<2, 2>(0, 0) =
        Eigen::Matrix<double, 2, 2>::Identity() * covRollPitch;
    /* Yaw has large initial uncertainty */
    covariance_(2, 2) = 1000 * M_PI / 180.0;
    double covGyroBias = (sigmaGyroWhiteNoise_ * sigmaGyroWhiteNoise_) / dataHZ;
    covariance_.block<3, 3>(3, 3) =
        Eigen::Matrix<double, 3, 3>::Identity() * covGyroBias;

    isEKFInitialized_ = true;

    return CFE_SUCCESS;
}

}   // namespace orientation_ekf