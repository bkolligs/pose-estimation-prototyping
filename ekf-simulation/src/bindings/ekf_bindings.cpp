#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "ekf.h"

using namespace orientation_ekf;

namespace py = pybind11;

PYBIND11_MODULE(ekf, m) {
	py::class_<OrientationEKF>(m, "OrientationEKF")
		.def(py::init<>())
		.def("is_ekf_initialized", &OrientationEKF::isEKFInitialized, "Checks whether the EKF is initialized")
		.def("handle_imu", &OrientationEKF::handleIMU, "Measurement update for the IMU")
		.def("handle_sun_sensor", &OrientationEKF::handleSunSensor, "Measurement update for the sun sensor")
		.def("compute_f", &OrientationEKF::computeF)
		.def("compute_g", &OrientationEKF::computeG)
		.def("compute_phi_and_qdk", &OrientationEKF::computePhiAndQdk)
		.def("get_state", &OrientationEKF::getState, py::return_value_policy::reference_internal)
		.def("get_f", &OrientationEKF::getF)
		.def("get_g", &OrientationEKF::getG)
		.def("get_phi", &OrientationEKF::getPhi)
		.def("get_ori", &OrientationEKF::getOriValues)
		.def("set_imu_count", &OrientationEKF::setIMUCount)
		.def("get_imu_count", &OrientationEKF::getIMUCount)
		;
}