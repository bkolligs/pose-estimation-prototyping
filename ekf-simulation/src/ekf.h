#include <math.h>

#include <chrono>
#include <string>

#include "Dense"
#include "Core"
#include "unsupported/Eigen/MatrixFunctions"

#include "cfs_simulation.h"

namespace orientation_ekf {

typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Quaternion<double> Quaternion;
/* EKF specific matrix sizes */
typedef Eigen::Matrix<double, 10, 1> StateVector;
typedef Eigen::Matrix<double, 9, 9> CovarianceMatrix;
typedef Eigen::Matrix<double, 12, 12> NoiseIMUMatrix;
typedef Eigen::Matrix<double, 3, 3> NoiseAccMatrix;
typedef Eigen::Matrix<double, 3, 3> NoiseMeasMatrix;
/* Unit axes used in initialization */
const Eigen::Matrix<double, 3, 1> unitX{1, 0, 0};
const Eigen::Matrix<double, 3, 1> unitY{0, 1, 0};
const Eigen::Matrix<double, 3, 1> unitZ{0, 0, 1};

typedef int32_t int32;

/* In order to maintain which sensor we use */
typedef enum
{
    SUN_SENSOR,
    ACCELEROMETER
} Sensor;

/**
 * @brief This class is to be used for determining the time step between
 * measurements
 */
class Timer {
   private:
    /* Last recorded time, and current time */
    std::chrono::steady_clock::time_point lastTime_, curTime_;
    /* Delta T for calculating integrations */
    float deltaT_;

   public:
    /**
     * @brief This function calculates the time step between the current time
     * and last time we count time
     */
    void calculateDeltaT(void);

    /**
     * @brief Construct a timer object, initializing the last time to now()
     */
    Timer() {
        /* Should this be mission time with cFS? */
        lastTime_ = std::chrono::steady_clock::now();
    }

    float updateAndGetDeltaT() {
        calculateDeltaT();
        return deltaT_;
    }

    void setlastTime(std::chrono::steady_clock::time_point newVal) {
        lastTime_ = newVal;
    }

    void setcurTime(std::chrono::steady_clock::time_point newVal) {
        curTime_ = newVal;
    }

    void setDeltaT(float newVal) {
        deltaT_ = newVal;
    }

    std::chrono::steady_clock::time_point getlastTime() {
        return lastTime_;
    }

    std::chrono::steady_clock::time_point getcurTime() {
        return curTime_;
    }

    float getDeltaT() {
        return deltaT_;
    }
};

/**
 * @brief This is the actual orientation Extended Kalman Filter
 */
class OrientationEKF {
   private:
    /* Noise Matrices */
    NoiseIMUMatrix noiseIMU_ = NoiseIMUMatrix::Zero();
    NoiseAccMatrix noiseAcc_ = NoiseAccMatrix::Zero();
    NoiseMeasMatrix noiseMeas_ = NoiseMeasMatrix::Zero();
    CovarianceMatrix covariance_ = CovarianceMatrix::Zero();

    /* Calculation matrices */
    Eigen::Matrix<double, 3, 3> specForceSkew_;
    Eigen::Matrix<double, 9, 9> F_ = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 9, 12> G_ = Eigen::Matrix<double, 9, 12>::Zero();
    Eigen::Matrix<double, 18, 18> A_ = Eigen::Matrix<double, 18, 18>::Zero();
    Eigen::Matrix<double, 9, 9> Phi_ = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 9, 9> Qdk_ = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 3, 9> Ha_ = Eigen::Matrix<double, 3, 9>::Zero();
    Eigen::Matrix<double, 3, 9> Hs_ = Eigen::Matrix<double, 3, 9>::Zero();

    /* We use the gravity vectors to determine roll/pitch */
    Vector3 predGravity_;
    Vector3 predGravitySum_;
    Vector3 trueGravity_;

    /* State vector [orientation, gyro bias, accel bias]^T */
    StateVector state_;
    /* Gravity default is earth gravity*/
    double gravity_ = 9.80665;
    /* Time step */
    double deltaT_;
    /* Parameters needed for initialization */
    bool isEKFInitialized_ = false;
    bool receivedIMUInitData_ = false;
    double numInitIMUData_ = 500;
    int imuCount_ = 0;
    int rateIMU_ = 125;
    Vector3 initSumAccel_;
    Vector3 initSumGyro_;
    /* Initial noise values */
    double sigmaGyro_ = 2.9E-6;
    double sigmaGyroWhiteNoise_ = 6.8585E-4;
    double sigmaAcc_ = 1.483E-5;
    double sigmaAccWhiteNoise_ = 2.20313E-3;
    /* Number of consecutive accelerometer measurements to declare robot is
     * stationary */
    int stationaryCounts = 125;
    bool isRoverStationary = false;
    /* Acceleration threshold */
    double accelerationThresh_ = 0.1;
    int accelerationCount_ = 0;
    /* Noise parameters for random walk */
    double noiseLambdaG;
    double noiseLambdaA;

    /* Timer object */
    Timer timer_;

   public:
    /**
     * @brief returns true if the matrix passed in is indeed a rotation matrix
     */
    static bool is_valid_rotation_matrix(Eigen::Matrix<double, 3, 3> R);

    /**
     * @brief Handles incoming sun sensor measurements
     * @param sunSensorData the estimate of the rover's orientation relative to
     * the sun
     */
    void handleSunSensor(const Vector3 &measSunSensor);

    /**
     * @brief hanldes incoming IMU measurements
     */
    int32 handleIMU(const Vector3 &measAng, const Vector3 &measAcc);

    void handleIMUInitialization(const Vector3 &measAng,
                                 const Vector3 &measAcc);

    /**
     * @brief Computes the F matrix in the equation dx_dot = F*dx + G*w
     */
    int32 computeF(const Vector3 &specForce,
                   const Eigen::Matrix<double, 3, 3> &RBodyToNavNext);

    /**
     * @brief Computes the G matrix in the equation dx_dot = F*dx + G*w
     * G is the linear transformation from gaussian noise in the IMU readings to
     * gaussian noise in the error of the IMU biases
     */
    int32 computeG(const Eigen::Matrix<double, 3, 3> RBodyToNavNext);

    /**
     * @brief Computes the discrete time state update matrix phi
     */
    int32 computePhiAndQdk(const Vector3 &f_i,
                           const Eigen::Matrix<double, 3, 3> &RBodyToNavNext);

    int32 stationaryMeasurementUpdate(
        const Eigen::Matrix<double, 3, 3> &RBodyToNav);

    void measurementUpdate(const Eigen::Matrix<double, 2, 9> &H,
                           const Eigen::Matrix<double, 2, 2> &R,
                           const Eigen::Vector2d &z,
                           const Sensor sensorType);

    /**
     * @brief converst a 3x1 vector to a skew symmetric matrix
     */
    void toSkew(const Vector3 &v, Eigen::Matrix<double, 3, 3> &vOut) {
        vOut << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    }

    OrientationEKF();

    /* Initialize the EKF */
    int32 initializeEKF();

    /* Get a constant reference to the state vector */
    const StateVector &getState() {
        return state_;
    }

    /**
     * @brief Sets all the parameters of the EKF from the configuration table
     * @param table pointer to the pe_config.tbl object
     */
    bool isEKFInitialized() {
        return isEKFInitialized_;
    }

    void setnoiseLambdaG(double lambdaG);

    void setnoiseLambdaA(double lambdaA);

    Eigen::Matrix<double, 9, 9> getF();

    Eigen::Matrix<double, 9, 12> getG();

    Eigen::Matrix<double, 9, 9> getPhi();

    Eigen::Matrix<double, 9, 9> getQdk();

    Eigen::Quaternion<double> getOriQuat();

    Eigen::Matrix<double, 4, 1> getOriValues();
};

}   // namespace orientation_ekf