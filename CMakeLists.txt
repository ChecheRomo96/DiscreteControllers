######################################################################################################
# Setting up the project

	##################################################################################################
	# C Make Minimum Version
		
		cmake_minimum_required(VERSION 3.14)
	#
	##################################################################################################
	# Project Name Version and Languaje, print and add preprocessor flag
		
		project(DiscreteControllers VERSION 0.0.1 LANGUAGES CXX)
		message(STATUS "Building ${PROJECT_NAME} Version = ${PROJECT_VERSION}...")

		set(PROJECT_NAME_UPPER DISCRETE_CONTROLLERS)
		#string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
	#
	##################################################################################################
	# Set up C++ Version

		set(CMAKE_CXX_STANDARD 17) 
		set(CMAKE_CXX_STANDARD_REQUIRED ON)
	#
	##################################################################################################
#
######################################################################################################
# Store this directory as a variable called MCC_SRC_DIRECTORY to reference in deeper directories

	set( ${PROJECT_NAME_UPPER}_ROOT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} )
#
######################################################################################################
# Compile Configuration

	set(${PROJECT_NAME_UPPER}_EXAMPLES FALSE CACHE BOOL "") # Enables Building Examples
	set(${PROJECT_NAME_UPPER}_TESTING FALSE CACHE BOOL "") # Enabled Unit Testing
#
######################################################################################################
# Append Source

	add_subdirectory(src)
	target_compile_definitions(DiscreteControllers INTERFACE ${PROJECT_NAME_UPPER}_VERSION="${PROJECT_VERSION}" )
#
######################################################################################################
# Append Examples

	if( ${PROJECT_NAME_UPPER}_EXAMPLES)
		message("Building ${CMAKE_PROJECT_NAME} Examples...")
		add_subdirectory(examples)
	endif()
#
######################################################################################################
# Append Tests

	if( ${${PROJECT_NAME_UPPER}_TESTING} )
		message("Building ${CMAKE_PROJECT_NAME} Tests...")

		cmake_policy(SET CMP0135 NEW)

		include(FetchContent)

		FetchContent_Declare(
		  googletest
		  URL https://github.com/google/googletest/archive/f8d7d77c06936315286eb55f8de22cd23c188571.zip
		)
		# For Windows: Prevent overriding the parent project's compiler/linker settings
		set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
		FetchContent_MakeAvailable(googletest)

		enable_testing()
		include(GoogleTest)
		set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${${PROJECT_NAME_UPPER}_ROOT_DIRECTORY}/bin")
		add_subdirectory(tests)
	endif()
#
######################################################################################################
