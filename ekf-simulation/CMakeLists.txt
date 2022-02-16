cmake_minimum_required(VERSION 3.16)
project(EKF_SIMULATION)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(PYTHON_PACKAGE_NAME "moonranger")

# Save the resulting library in the pose-estimation-prototyping/lib folder
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "../libs")

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

# Now install the package to the current python interpreter
find_package(Python3 REQUIRED)
set(CMAKE_INSTALL_PREFIX ${Python3_SITELIB})

set(PYTHON_PACKAGE_FILES
	libs/__init__.py
)

# Install the pybind11-produced library inside the current python interpreter's sitepackages directory
install(TARGETS ekf DESTINATION ${PYTHON_PACKAGE_NAME})
install(FILES ${PYTHON_PACKAGE_FILES} DESTINATION ${PYTHON_PACKAGE_NAME})


# Print out the variables in the current CMAKE context
set(PRINT_VARIABLES FALSE CACHE BOOL "Print out the varialbes in the current CMake context")
if (${PRINT_VARIABLES})
	# Prints out all the availiable CMAKE Variables for debugging
	get_cmake_property(_variableNames VARIABLES)
	list (SORT _variableNames)
	foreach (_variableName ${_variableNames})
		message(STATUS "${_variableName}=${${_variableName}}")
	endforeach()
endif()