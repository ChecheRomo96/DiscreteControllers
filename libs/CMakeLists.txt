######################################################################################################
# CPSTL 
    
    if(NOT TARGET CPSTL)
        if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/CPSTL)
            set( CPSTL_EXAMPLES FALSE CACHE BOOL "")
            set( CPSTL_TESTING FALSE CACHE BOOL "")
            set( CPSTL_INSTALL FALSE CACHE BOOL "")

            message("Using the CPSTL Source on: MusicCompositonCore/libs/CPSTL")
            add_subdirectory(CPSTL)
        else()

            find_package(CPSTL)

            if(CPSTL_FOUND)
                message("Using the CPSTL Package installed on: ${CMAKE_INSTALL_PREFIX}")
            else()
                message(FATAL_ERROR "CPSTL not found.")
            endif()
        endif()
    endif()

    target_link_libraries( DiscreteControllers INTERFACE CPSTL )
#
######################################################################################################
