cmake_minimum_required(VERSION 3.10)
project(UWScan LANGUAGES CXX)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    # 启用 C++ 异常处理
    add_compile_options(/Zi /EHsc /FS /arch:AVX2 /bigobj /Zm200)
    add_compile_definitions(
                        GLOG_NO_ABBREVIATED_SEVERITIES
                        NOMINMAX
                        _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
                        _CRT_SECURE_NO_WARNINGS 
                        _CRT_NONSTDC_NO_DEPRECATE
                        _HAS_STD_BYTE=0
                        )
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
# 自动处理Qt的元对象系统
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(UWSCAN_BUILD_DIR "${PROJECT_SOURCE_DIR}/build")
set(UWSCAN_BUILD_LIB_DIR "${UWSCAN_BUILD_DIR}/lib")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${UWSCAN_BUILD_LIB_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${UWSCAN_BUILD_LIB_DIR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
foreach(config Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${config}" config_upper)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${config_upper} "${UWSCAN_BUILD_LIB_DIR}/${config}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${config_upper} "${UWSCAN_BUILD_LIB_DIR}/${config}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_upper} ${PROJECT_SOURCE_DIR}/bin)
endforeach()
# UI文件的查找地址
set(CMAKE_AUTOUIC_SEARCH_PATHS ${PROJECT_SOURCE_DIR}/resources/ui)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# 查找Qt5
set(Qt5_DIR "C:\\Qt\\5.15.2\\msvc2019_64\\lib\\cmake\\Qt5")
find_package(Qt5 COMPONENTS Core Gui Qml Quick Widgets Network SerialPort SerialBus REQUIRED)
get_filename_component(QT_ROOT_DIR "${Qt5_DIR}/../../.." ABSOLUTE)
find_program(QT_WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${QT_ROOT_DIR}/bin")
# 配置第三方库
set(PCL_DIR "D:\\Program Files\\PCL 1.12.1\\cmake")
find_package(PCL REQUIRED)

if(MSVC)
    foreach(pcl_target IN LISTS PCL_LIBRARIES)
        if(TARGET "${pcl_target}")
            get_target_property(pcl_defs "${pcl_target}" INTERFACE_COMPILE_DEFINITIONS)
            if(pcl_defs)
                list(FILTER pcl_defs EXCLUDE REGEX "BOOST_ALL_NO_LIB-DBOOST_ALL_NO_LIB")
                list(APPEND pcl_defs BOOST_ALL_NO_LIB)
                list(REMOVE_DUPLICATES pcl_defs)
                set_target_properties("${pcl_target}" PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${pcl_defs}")
            endif()
        endif()
    endforeach()
endif()

# find_package(LASTools REQUIRED)
set(CMAKE_MAP_IMPORTED_CONFIG_DEBUG Release)
# find_package(pylon REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(UW3DLIB REQUIRED)

find_package(SARibbon REQUIRED)

set(BASLER_PYLON_ROOT "D:\\Program Files\\Basler\\pylon 7")
set(BASLER_PYLON_INCLUDE_DIR "${BASLER_PYLON_ROOT}/Development/include")
set(BASLER_PYLON_LIB_DIR "${BASLER_PYLON_ROOT}/Development/lib/x64")
set(BASLER_PYLON_RUNTIME_DIR "${BASLER_PYLON_ROOT}/Runtime/x64")

include_directories("${BASLER_PYLON_INCLUDE_DIR}")
link_directories("${BASLER_PYLON_LIB_DIR}")
include_directories(${SARIBBON_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

if(NOT OpenCV_INCLUDE_DIRS)
    set(OpenCV_INCLUDE_DIRS "${OpenCV_DIR}/include")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(ALL_LIBS
    Qt5::Core 
    Qt5::Gui 
    Qt5::Qml 
    Qt5::Quick 
    Qt5::Widgets 
    Qt5::Network 
    Qt5::SerialPort
    Qt5::SerialBus
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    # LASTools::LASlib 
    # pylon::pylon
    UW3DLIB::UW3DLIB
    Ceres::Ceres
    SARibbon::SARibbon
)

add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
