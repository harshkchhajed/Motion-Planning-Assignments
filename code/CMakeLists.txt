cmake_minimum_required(VERSION 3.10)

project(Project4)

set(CMAKE_CXX_STANDARD 11)

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

# Build the car executable: includes car.cpp, RG-RRT.cpp, and CollisionChecking.cpp
add_executable(car car.cpp RG-RRT.cpp CollisionChecking.cpp)
target_link_libraries(car ${OMPL_LIBRARIES})

# Build the pendulum executable: includes pendulum.cpp, RG-RRT.cpp, and CollisionChecking.cpp (if needed)
add_executable(pendulum pendulum.cpp RG-RRT.cpp CollisionChecking.cpp)
target_link_libraries(pendulum ${OMPL_LIBRARIES})
