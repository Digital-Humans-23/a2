cmake_minimum_required(VERSION 3.11)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB CRL_SOURCES #
        "src/*.cpp" #
)

pybind11_add_module(
        ${CRL_TARGET_NAME} #
        ${CRL_SOURCES} #
)

IF (APPLE)
    SET_TARGET_PROPERTIES(${CRL_TARGET_NAME}
            PROPERTIES
            LINK_FLAGS "-undefined dynamic_lookup"
            SUFFIX ".so"
            )
ENDIF ()

IF (UNIX AND NOT APPLE)
    set_target_properties(${CRL_TARGET_NAME} PROPERTIES PREFIX "")
    set_target_properties(${CRL_TARGET_NAME} PROPERTIES SUFFIX ".so")
endif ()

IF (WIN32)
    set_target_properties(${CRL_TARGET_NAME} PROPERTIES PREFIX "")
    set_target_properties(${CRL_TARGET_NAME} PROPERTIES SUFFIX ".pyd")
endif ()

set_target_properties(${CRL_TARGET_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# target dependencies
add_dependencies(
        ${CRL_TARGET_NAME} #
        crl::pylocobase
)

# target include dirs
target_include_directories(
        ${CRL_TARGET_NAME} #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# target link libs
target_link_libraries(
        ${CRL_TARGET_NAME} #
        PUBLIC "crl::pylocobase" #
)

# compile definitions
target_compile_definitions(
        ${CRL_TARGET_NAME}
        PUBLIC "PYLOCO_DATA_FOLDER=${PYLOCO_DATA_FOLDER}" #
)

# generate python file contains library path
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(PYLOCO_LIB_PATH ${CMAKE_CURRENT_BINARY_DIR}/${CRL_TARGET_NAME}.so)
    configure_file(cmake_variables.py.in ${PROJECT_SOURCE_DIR}/src/python/pylocogym/cmake_variables.py @ONLY)
endif ()