# Automatically generated by scripts/boost/generate-ports.ps1

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO boostorg/conversion
    REF boost-1.77.0
    SHA512 d0c866f7cf01be8bd98903a5ea92f678eb119f767b97caa8a2e5edaafd2cfbe838ee02c50301de0a9cf8082db95e3379f2fb1cd2b7ed835288c89171b2753da2
    HEAD_REF master
)

include(${CURRENT_INSTALLED_DIR}/share/boost-vcpkg-helpers/boost-modular-headers.cmake)
boost_modular_headers(SOURCE_PATH ${SOURCE_PATH})
