cmake_minimum_required(VERSION 3.19)
project(mono_orb_slam)

find_package(OpenCV QUIET)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

set(CMAKE_CXX_STANDARD 14)
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIR})

#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED
        src/System.cpp 
        src/Tracking.cpp
src/ORBextractor.cpp 
src/Frame.cpp 
src/ORBVocabulary.cpp)
        
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBS})

add_executable(mono_orb Example/monocular/main.cpp)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Example/monocular)
target_link_libraries(mono_orb ${PROJECT_NAME})