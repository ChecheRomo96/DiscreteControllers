add_subdirectory(1_BuildConfiguration)

if( ${CPSTL_VECTOR_STATUS}  AND ${CPSTL_VECTOR_EXAMPLES})

    message("Appending CPSTL vector examples")
    add_subdirectory(vector)

endif() 