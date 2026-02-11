# Define minimal required version of CMake.
cmake_minimum_required(VERSION "3.25")

# Project definition
project(
	oxystat_quad
	VERSION 1.0
	LANGUAGES C CXX ASM
)

# Include the definition of the slc_oxystat_quad target,
# which contains the content of the SLC project
include(oxystat_quad.cmake)

add_executable(oxystat_quad
    # Add additional sources here
    ../blink.c
    ../max5535.c
    ../oxystat_service.c
    ../oxystat.c
    ../utilities.c
    ../flexiBLE_common.c
    ../oxyconfig.c
)

target_include_directories(oxystat_quad PUBLIC
    # Add additional include paths here
)

target_compile_definitions(oxystat_quad PUBLIC
    # Add additional macros here
)

target_compile_options(oxystat_quad PUBLIC
    # Set additional compiler flags here
)

target_link_options(oxystat_quad PUBLIC
    # Set additional linker flags here
)

# Link with the content defined in the SLC project
target_link_libraries(oxystat_quad PRIVATE
    slc_oxystat_quad
)

# Include managed project content if available
include(oxystat_quad_project.cmake OPTIONAL RESULT_VARIABLE managed_project)
if(managed_project)
    message(STATUS "Using managed project content from ${managed_project}")
endif()

# Force the gcc linker command
set_target_properties(oxystat_quad PROPERTIES LINKER_LANGUAGE C)

# Create .bin, .hex and .s37 artifacts after building the project
add_custom_command(TARGET oxystat_quad
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O srec "$<TARGET_FILE:oxystat_quad>" "$<TARGET_FILE_DIR:oxystat_quad>/$<TARGET_FILE_BASE_NAME:oxystat_quad>.s37"
    COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:oxystat_quad>" "$<TARGET_FILE_DIR:oxystat_quad>/$<TARGET_FILE_BASE_NAME:oxystat_quad>.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:oxystat_quad>" "$<TARGET_FILE_DIR:oxystat_quad>/$<TARGET_FILE_BASE_NAME:oxystat_quad>.bin" 
)

# Run post-build pipeline to perform additional post-processing
if(post_build_command)
add_custom_command(TARGET oxystat_quad
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMAND ${post_build_command}
)
endif()