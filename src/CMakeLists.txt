# Store this directory as a variable called DISCRETE_CONTROLLERS_SRC_DIRECTORY to reference in deeper directories
set( DISCRETE_CONTROLLERS_SRC_DIRECTORY  ${CMAKE_CURRENT_LIST_DIR})

# Initialize HEADER_LIST as an empty string list
set( HEADERS_LIST "" )

# Append Headers
list( APPEND HEADERS_LIST "${CMAKE_CURRENT_LIST_DIR}/DiscreteControllers.h")
list( APPEND HEADERS_LIST "${CMAKE_CURRENT_LIST_DIR}/DiscreteControllers_UserSetup.h")
list( APPEND HEADERS_LIST "${CMAKE_CURRENT_LIST_DIR}/DiscreteControllers_BuildSettings.h")

# Add Subdirectories
add_subdirectory(FIR)
add_subdirectory(IIR)

# Create the DiscreteControllers library (STATIC is not justified tho) and include the directories needed to compile
add_library(DiscreteControllers INTERFACE)

# Include directories for install and for build 
target_include_directories( 
	DiscreteControllers INTERFACE 
	$<INSTALL_INTERFACE:include/DiscreteControllers> 
	$<BUILD_INTERFACE:${DISCRETE_CONTROLLERS_SRC_DIRECTORY}> 
)

# DiscreteControllers install process
if( INSTALL STREQUAL "ON")

	install(
		FILES ${HEADERS_LIST} 
		DESTINATION ${CMAKE_INSTALL_PREFIX}/include/DiscreteControllers 
	)

	install(
		TARGETS DiscreteControllers 
		EXPORT DiscreteControllersExport
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/DiscreteControllers
	)

	install(
		EXPORT DiscreteControllersExport
		FILE DiscreteControllers-config.cmake
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/DiscreteControllers
	)

endif()