find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)

set(SOURCE_FILES
    ConfigSettings.cpp
)
set(EXEC_NAME ConfigSettings)

add_library(${EXEC_NAME} ${SOURCE_FILES})

target_include_directories(${EXEC_NAME} PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    include
)

target_link_libraries(${EXEC_NAME} PUBLIC
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)
