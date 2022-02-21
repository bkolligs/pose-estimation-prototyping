cmake_minimum_required(VERSION 3.16)
project(EKF_SIMULATION)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(PYTHON_PACKAGE_NAME "moonranger")

# Save the resulting library in the pose-estimation-prototyping/lib folder
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs")

# We're going to be binding the C++ to python
find_package(pybind11 REQUIRED)
pybind11_add_module(
	ekf
	src/ekf
	src/bindings/ekf_bindings
)

# Include the files for binding and eigen path here
target_include_directories(ekf PUBLIC 
	#  For the bindings
	src/
	# For eigen
	libs/eigen/Eigen
	libs/eigen
)

set(PYTHON_PACKAGE_FILES
	libs/__init__.py
)

# Install the pybind11-produced library inside the current python interpreter's sitepackages directory
install(TARGETS ekf DESTINATION ${PYTHON_PACKAGE_NAME})
install(FILES ${PYTHON_PACKAGE_FILES} DESTINATION ${PYTHON_PACKAGE_NAME})