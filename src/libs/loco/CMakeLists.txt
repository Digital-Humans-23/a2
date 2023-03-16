cmake_minimum_required(VERSION 3.11)

project(loco)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB_RECURSE
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

# basic dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "crl::utils" #
        "crl::gui" #
        "ODE" #
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
        )

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC "crl::utils" #
        PUBLIC "crl::gui" #
        PUBLIC "ODE" #
)

# compile definitions
list(
        APPEND CRL_COMPILE_DEFINITIONS #
        PUBLIC "CRL_LOCO_DATA_FOLDER=${CRL_LOCO_DATA_FOLDER}" #
)

# create target
create_crl_library(
        ${CRL_TARGET_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}"
)