cmake_minimum_required(VERSION 3.20)

project(FalconBridge LANGUAGES CXX)

# DHD SDK location
set(DHD_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../ForceDimensionSDK/sdk-3.17.7"
)

# DHD header
set(DHD_INCLUDE_DIR
    "${DHD_ROOT}/include"
)

# DHD ARM64 library
set(DHD_LIBRARY
    "${DHD_ROOT}/lib/release/mac-arm64-clang/libdhd.3.17.7.dylib"
)

# Check that the SDK files exist
if(NOT EXISTS "${DHD_INCLUDE_DIR}/dhdc.h")
    message(FATAL_ERROR "Cannot find dhdc.h")
endif()

if(NOT EXISTS "${DHD_LIBRARY}")
    message(FATAL_ERROR "Cannot find DHD library")
endif()

# Create a macOS plugin bundle
add_library(FalconBridge MODULE
    src/FalconBridge.cpp
)

set_target_properties(FalconBridge PROPERTIES
    BUNDLE TRUE
    BUNDLE_EXTENSION "bundle"
    PREFIX ""
    OUTPUT_NAME "FalconBridge"
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Include DHD header
target_include_directories(FalconBridge PRIVATE
    "${DHD_INCLUDE_DIR}"
)

# Link DHD library
target_link_libraries(FalconBridge PRIVATE
    "${DHD_LIBRARY}"
)

# SDK library location for development-time loading
set_target_properties(FalconBridge PROPERTIES
    BUILD_RPATH "${DHD_ROOT}/lib/release/mac-arm64-clang"
)

find_package(Threads REQUIRED)

target_link_libraries(FalconBridge
    PRIVATE
    "${FD_LIBRARY}"
    Threads::Threads
)