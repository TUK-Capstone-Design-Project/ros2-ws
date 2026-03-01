find_package(OpenCV REQUIRED)

set(SOURCE_FILES
    ImgPreprocessor.cpp
)

add_library(ImgPreprocessor ${SOURCE_FILES})

target_include_directories(ImgPreprocessor PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ImgPreprocessor PUBLIC
    ${OpenCV_LIBS}
    ConfigSettings
    DotDetector
    LineProcessor
)
