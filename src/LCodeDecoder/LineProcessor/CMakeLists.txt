find_package(OpenCV REQUIRED)

set(LIB_NAME LineProcessor)
set(SOURCE_FILES
    LineProcessor.cpp
)

add_library(${LIB_NAME} ${SOURCE_FILES})

target_include_directories(${LIB_NAME} PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${LIB_NAME} PUBLIC
    ${OpenCV_LIBS}
    ConfigSettings
    DotDetector
)
