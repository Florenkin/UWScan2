# cmake/FindCeres.cmake
# Find Ceres library
#
# This module defines:
#  CERES_FOUND - true if Ceres is found
#  CERES_INCLUDE_DIRS - Ceres include directories
#  CERES_LIBRARIES - Ceres libraries to link against

# 设置Ceres的根目录
if(NOT CERES_ROOT)
set(CERES_ROOT "${CMAKE_SOURCE_DIR}/lib/ceresLib")
endif()

find_path( 
    CERES_INCLUDE_DIR
    NAMES ceres/ceres.h
    PATHS 
        "${CERES_ROOT}/ceres/include"
    DOC "Ceres include directory"
)

find_path(
    CERES_EIGEN_INCLUDE_DIR
    NAMES Eigen/Core
    PATHS 
        "${CERES_ROOT}/Eigen"
    DOC "Ceres Eigen include directory"
)

find_path(
    CERES_GFLAGS_INCLUDE_DIR
    NAMES gflags/gflags.h
    PATHS 
        "${CERES_ROOT}/gflags/include"
    DOC "Ceres gflags include directory"
)

find_path(
    CERES_GLOG_INCLUDE_DIR
    NAMES glog/logging.h
    PATHS 
        "${CERES_ROOT}/glog/include"
    DOC "Ceres glog include directory"
)

find_library(
    CERES_LIBRARY
    NAMES ceres
    PATHS 
        "${CERES_ROOT}/ceres/lib"
    DOC "Ceres library"
)

find_library(
    CERES_GFLAGS_LIBRARY
    NAMES gflags
    PATHS 
        "${CERES_ROOT}/gflags/lib"
    DOC "Ceres gflags library"
)

find_library(
    CERES_GLOG_LIBRARY
    NAMES glog
    PATHS 
        "${CERES_ROOT}/glog/lib"
    DOC "Ceres glog library"
)

# 处理查找结果
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ceres
    FOUND_VAR CERES_FOUND
    REQUIRED_VARS 
        CERES_INCLUDE_DIR
        CERES_EIGEN_INCLUDE_DIR
        CERES_GFLAGS_INCLUDE_DIR
        CERES_GLOG_INCLUDE_DIR
        CERES_LIBRARY
        CERES_GFLAGS_LIBRARY
        CERES_GLOG_LIBRARY
)

if(CERES_FOUND)
    # 设置输出变量
    set(CERES_INCLUDE_DIRS 
        ${CERES_INCLUDE_DIR}
        ${CERES_EIGEN_INCLUDE_DIR}
        ${CERES_GFLAGS_INCLUDE_DIR}
        ${CERES_GLOG_INCLUDE_DIR}
    )

    set(CERES_LIBRARIES ${CERES_LIBRARY} ${CERES_GFLAGS_LIBRARY} ${CERES_GLOG_LIBRARY})
    
    # 创建导入目标（现代CMake方式）
    if(NOT TARGET Ceres::Ceres)
        add_library(Ceres::Ceres UNKNOWN IMPORTED)
        set_target_properties(Ceres::Ceres PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${CERES_INCLUDE_DIRS}"
            IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
            IMPORTED_LOCATION "${CERES_LIBRARY}"
            IMPORTED_LOCATION_DEBUG "${CERES_LIBRARY}"
            IMPORTED_LOCATION_RELEASE "${CERES_LIBRARY}"
        )
    endif()
    
    # 打印状态信息
    message(STATUS "Found Ceres:")
    message(STATUS "  Includes: ${CERES_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${CERES_LIBRARIES}")
endif()

mark_as_advanced(
    CERES_INCLUDE_DIR
    CERES_EIGEN_INCLUDE_DIR
    CERES_GFLAGS_INCLUDE_DIR
    CERES_GLOG_INCLUDE_DIR
    CERES_LIBRARY
    CERES_GFLAGS_LIBRARY
    CERES_GLOG_LIBRARY
)
