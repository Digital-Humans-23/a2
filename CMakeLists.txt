# we need cmake >= 3.11 to support fetch_content that downloads dependencies
cmake_minimum_required(VERSION 3.11)

# set name of the project
project(pyloco)

set(CMAKE_CXX_STANDARD 17)

# this option is necessary since we need to build a shared library for python
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# release as a default build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
            "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# data folder path
set(CRL_DATA_FOLDER
    "\"${CMAKE_CURRENT_LIST_DIR}/data\""
    CACHE STRING ""
)

# data folder path
set(CRL_LOCO_DATA_FOLDER
        "\"${CMAKE_CURRENT_LIST_DIR}/data\""
        CACHE STRING ""
        )

# data folder path
set(PYLOCO_DATA_FOLDER
        "\"${CMAKE_CURRENT_LIST_DIR}/data\""
        CACHE STRING ""
        )

# log folder path
set(PYLOCO_LOG_FOLDER
        "\"${CMAKE_CURRENT_LIST_DIR}/log\""
        CACHE STRING ""
        )

# -----------------------------------------------------------------------------
# options
# -----------------------------------------------------------------------------

option(CRL_BASIC_BUILD_APPS "Build crl-basic example apps." OFF)
option(BUILD_TESTS "Build unit tests" OFF)
option(PYLOCO_BUILD_APPS "Build pyloco apps." ON)

# -----------------------------------------------------------------------------
# unit testing
# -----------------------------------------------------------------------------

if(BUILD_TESTS)
    enable_testing()
    include(GoogleTest)
endif()

# -----------------------------------------------------------------------------
# macro and functions
# -----------------------------------------------------------------------------

# create static library named LIB_NAME
function(
    create_crl_library #
    LIB_NAME #
    SOURCE #
    DEPENDENCY #
    INCLUDE_DIRS #
    LINK_LIBS #
    COMPILE_DEFINITIONS #
)
    add_library(${LIB_NAME} STATIC ${SOURCE})
    add_library(crl::${LIB_NAME} ALIAS ${LIB_NAME})
    add_dependencies(${LIB_NAME} ${DEPENDENCY})
    target_include_directories(${LIB_NAME} ${INCLUDE_DIRS})
    target_link_libraries(${LIB_NAME} ${LINK_LIBS})

    if(COMPILE_DEFINITIONS)
        # scope is PUBLIC so these definitions are propagated
        target_compile_definitions(${LIB_NAME} ${COMPILE_DEFINITIONS})
    endif()

    # For solution explorer in visual studios
    set_property(TARGET ${LIB_NAME} PROPERTY FOLDER "Libs")
endfunction()

# create executable named APP_NAME
function(
    create_crl_app #
    APP_NAME #
    SOURCE #
    DEPENDENCY #
    INCLUDE_DIRS #
    LINK_LIBS #
    COMPILE_DEFINITIONS #
    )
    
    add_executable(${APP_NAME} ${SOURCE})
    add_dependencies(${APP_NAME} ${DEPENDENCY})
    target_include_directories(${APP_NAME} ${INCLUDE_DIRS})
    target_link_libraries(${APP_NAME} ${LINK_LIBS})

    if(COMPILE_DEFINITIONS)
        target_compile_definitions(${APP_NAME} ${COMPILE_DEFINITIONS})
    endif()

    # For solution explorer in visual studios
    set_property(TARGET ${APP_NAME} PROPERTY FOLDER "Apps")
endfunction()

# create test executable named TEST_NAME
function(
    create_crl_test #
    TEST_NAME #
    SOURCE #
    DEPENDENCY #
    INCLUDE_DIRS #
    LINK_LIBS #
    COMPILE_DEFINITIONS #
    )

    if(BUILD_TESTS)
        add_executable(${TEST_NAME} ${SOURCE})
        add_dependencies(${TEST_NAME} ${DEPENDENCY})
        target_include_directories(${TEST_NAME} ${INCLUDE_DIRS})
        target_link_libraries(${TEST_NAME} ${LINK_LIBS} gtest gtest_main)
        gtest_discover_tests(${TEST_NAME})

        if(COMPILE_DEFINITIONS)
            target_compile_definitions(${TEST_NAME} ${COMPILE_DEFINITIONS})
        endif()

        # For solution explorer in visual studios
        set_property(TARGET ${TEST_NAME} PROPERTY FOLDER "tests")
    endif()
endfunction()

# -----------------------------------------------------------------------------
# code
# -----------------------------------------------------------------------------

# thirdparty code
add_subdirectory(ext)

# our code
add_subdirectory(src)

# -----------------------------------------------------------------------------
# clean up solutions view
# -----------------------------------------------------------------------------
if(WIN32)
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
    set_target_properties(glfw PROPERTIES FOLDER Ext)
    set_target_properties(glad PROPERTIES FOLDER Ext)
    set_target_properties(imgui PROPERTIES FOLDER Ext)
    set_target_properties(tinyobjloader PROPERTIES FOLDER Ext)
endif(WIN32)
