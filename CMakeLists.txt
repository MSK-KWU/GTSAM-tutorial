# CMake 최소 요구 버전을 설정합니다.
cmake_minimum_required(VERSION 3.10)

# 프로젝트 이름과 버전을 설정합니다.
project(gtsam_tutorial)

# C++ 표준을 17로 설정합니다. 필요에 따라 11 또는 14로 변경할 수 있습니다.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GTSAM 라이브러리를 찾습니다.
# GTSAM을 시스템에 설치한 경로를 CMake가 찾지 못하는 경우,
# -DGTSAM_DIR=/path/to/gtsam/build/cmake 와 같이 경로를 직접 지정해주어야 할 수 있습니다.
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED)
# Eigen 라이브러리를 포함합니다. GTSAM은 Eigen에 의존성이 있습니다.
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 실행 파일(executable)을 빌드하기 위한 소스 파일을 지정합니다.
add_executable(gtsam_factor_graph gtsam_factor_graph.cpp)
add_executable(sfm_example sfm_example.cpp)

# 빌드할 실행 파일에 GTSAM 라이브러리를 링크합니다.
target_link_libraries(gtsam_factor_graph gtsam)
target_link_libraries(sfm_example gtsam)

# C++ 헤더 파일이 있는 디렉토리를 포함 경로에 추가합니다.
# gtsam_factor_graph.hpp 파일이 gtsam_factor_graph.cpp와 같은 디렉토리에 있으므로,
# 현재 디렉토리를 포함 경로에 추가합니다.
target_include_directories(gtsam_factor_graph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(sfm_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})