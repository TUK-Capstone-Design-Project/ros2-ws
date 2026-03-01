find_package(OpenCV REQUIRED)

set(MODULE_NAME DotDetector)
set(SOURCE_FILES
    DotDetector.cpp
)

add_library(${MODULE_NAME} ${SOURCE_FILES})

target_include_directories(${MODULE_NAME} PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${MODULE_NAME} PUBLIC
    ${OpenCV_LIBS}
)
