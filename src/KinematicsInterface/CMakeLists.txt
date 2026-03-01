find_package(OpenCV REQUIRED)

set(LIB_NAME KinematicsInterface)

set(SOURCE_FILES
    KinematicsInterface.cpp
)

add_library(${LIB_NAME} ${SOURCE_FILES})

target_include_directories(${LIB_NAME} PUBLIC
    include
)
