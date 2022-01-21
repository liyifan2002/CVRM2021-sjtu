# CMakeList.txt: CV2022 的 CMake 项目，在此处包括源代码并定义
cmake_minimum_required(VERSION 3.8)
project("CV2022")

set(CMAKE_CXX_STANDARD 20)
find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()

add_compile_options(-Wall -Werror=return-type -Wno-unused-variable)
if (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(-O3 -flto -march=native)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    add_compile_options(-O3 -flto -march=native -g)
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    add_compile_options(-Os -flto -march=native)
else ()
    add_compile_definitions(_DEBUG)
    add_compile_options(-O0 -g)
endif ()

find_package(OpenCV 4.5 REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Ceres REQUIRED)
find_package(fmt)

add_subdirectory(include/serial)

if (CAM STREQUAL DH)
    set(CAM_DRV Stream/DAHUA.cpp)
    set(CAM_LIB include/camera/DAHUA/lib/MVSDKmd.lib)
else ()
    set(CAM_DRV Stream/MindVision.cpp)
    set(CAM_LIB include/camera/MVSDK/lib/MVCAMSDK_X64.lib)
endif ()

if (EN_ENERGY)
    aux_source_directory(Energy Energy_SRCS)
endif ()

add_executable(${PROJECT_NAME}
        CV2022.cpp
        Stream/Stream.cpp
        Stream/record.cpp
        ${CAM_DRV}
        ${Energy_SRCS}
        )
target_include_directories(CV2022 PUBLIC
        ${GLOG_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        )

target_link_libraries(${PROJECT_NAME} PRIVATE
        ${OpenCV_LIBS}
        glog::glog gflags serial
        ${CERES_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/${CAM_LIB}
        )


