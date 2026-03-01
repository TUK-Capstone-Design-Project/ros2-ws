set(SOURCE_FILES
    Digitizer.cpp
)

find_package(OpenCV REQUIRED)

add_library(Digitizer ${SOURCE_FILES})

target_include_directories(Digitizer PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(Digitizer PUBLIC ${OpenCV_LIBS})