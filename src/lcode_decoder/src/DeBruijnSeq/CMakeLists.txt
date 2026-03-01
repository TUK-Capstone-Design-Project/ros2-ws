
set(SOURCE_FILES
    DeBruijnSeq.cpp
    GenBGArray.cpp
)

find_package(OpenCV REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/src/include
    ${OpenCV_INCLUDE_DIRS}
)

# Add library
add_library(DeBruijnSeq ${SOURCE_FILES})

# Link libraries
target_link_libraries(DeBruijnSeq PUBLIC ${OpenCV_LIBS})