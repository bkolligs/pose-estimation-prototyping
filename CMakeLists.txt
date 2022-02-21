cmake_minimum_required(VERSION 3.16)
project(PROTO_POSE)
add_subdirectory(ekf-simulation)

# Now install the package to the current python interpreter
find_package(Python3 REQUIRED)
set(CMAKE_INSTALL_PREFIX ${Python3_SITELIB})

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