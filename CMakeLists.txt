# CMake 최소 요구 버전을 설정합니다.
cmake_minimum_required(VERSION 3.10)

# 프로젝트 이름과 버전을 설정합니다.
project(gtsam_tutorial VERSION 1.0)

# C++ 표준을 17로 설정합니다.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GTSAM 라이브러리를 찾습니다.
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 소스 파일 경로 설정
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# 실행 파일(executable)을 빌드하기 위한 소스 파일을 지정합니다.
add_executable(gtsam_factor_graph ${SRC_DIR}/gtsam_factor_graph.cpp)
add_executable(sfm_example ${SRC_DIR}/sfm_example.cpp)

# 빌드할 실행 파일에 GTSAM 라이브러리를 링크합니다.
target_link_libraries(gtsam_factor_graph gtsam)
target_link_libraries(sfm_example gtsam)

# 헤더 파일이 있는 디렉토리를 포함 경로에 추가합니다.
target_include_directories(gtsam_factor_graph PRIVATE ${SRC_DIR})
target_include_directories(sfm_example PRIVATE ${SRC_DIR})

# 빌드된 실행 파일의 출력 디렉토리 설정
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)