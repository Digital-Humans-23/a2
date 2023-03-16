set(CRL_TARGET_NAME pylocobase)

file(
        GLOB_RECURSE
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

# basic dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "crl::loco" #
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
        )

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC "crl::loco" #
)

# compile definitions
list(
        APPEND CRL_COMPILE_DEFINITIONS #
        PUBLIC "PYLOCO_DATA_FOLDER=${PYLOCO_DATA_FOLDER}" #
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
