cmake_minimum_required(VERSION 3.0.2)
#project(dog_control)

find_package(Lua REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
SET(CMAKE_BUILD_TYPE "Release") 

file(GLOB_RECURSE DOG_CONTROL_SRCS "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")
file(GLOB_RECURSE DOG_CONTROL_INCS "${CMAKE_CURRENT_LIST_DIR}/include/*.h include/*.hpp")

set(DOG_CONTROL_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)

include_directories(
  ${DOG_CONTROL_INCLUDE_DIR}
  ${boost_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(dog_control_core STATIC
  ${DOG_CONTROL_SRCS}
)

target_link_libraries(dog_control_core
  ${Boost_LIBRARIES}
  ${LUA_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

set(DOG_CONTROL_LIBRARIES dog_control_core)

