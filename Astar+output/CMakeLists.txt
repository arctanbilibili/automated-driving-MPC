cmake_minimum_required(VERSION 3.0.0)
project(estun VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(estun main.cpp)

# set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -std=c90")
# set_target_properties(estun C_STANDARD 90)
# set(CMAKE_C_STANDARD 90)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
