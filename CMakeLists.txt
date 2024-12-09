cmake_minimum_required(VERSION 3.5)
project(AKFSFSimulation)

set(CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(SRC_CPP
    src/main.cpp
    src/readData.cpp
    src/simulation.cpp
    src/sensors.cpp
    src/utils.cpp
    src/kalmanfilter.cpp
    src/filterroad.cpp
)


set(ACCEL_STD 1)
set(GYRO_STD "0.02 / 180.0 * 3.141592653589793") # Use a string for expressions
set(INIT_VEL_STD 10)
set(C_A 0.1)
set(num_R2 0.009)
set(num_nG "0.5/180 * 3.141592653589793")
set(ACCEL_BIAS 0.001)
set(GYRO_BIAS "0.001 / 180.0 * 3.141592653589793")
set(ODO_STD 0.001)
set(ODO_BIAS 0.001)
set(COR 0.01)

# Generate the configuration header
configure_file(
    ${CMAKE_SOURCE_DIR}/config.h.in 
    ${CMAKE_BINARY_DIR}/config.h
    @ONLY
)




add_executable(${PROJECT_NAME}
        ${SRC_CPP}
        )

target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        )

add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/data/ $<TARGET_FILE_DIR:${PROJECT_NAME}>)
    
    