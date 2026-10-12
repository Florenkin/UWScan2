# cmake/FindLASTools.cmake
# Find LASTools library
#
# This module defines:
#  LASTOOLS_FOUND - true if LASTools is found
#  LASTOOLS_INCLUDE_DIRS - LASTools include directories
#  LASTOOLS_LIBRARIES - LASTools libraries to link against

# 设置LASTools的根目录
set(LASTOOLS_ROOT "" CACHE PATH "Path to LASTools root directory")
if(NOT LASTOOLS_ROOT)
    if(DEFINED ENV{LASTOOLS_ROOT})
        set(LASTOOLS_ROOT "$ENV{LASTOOLS_ROOT}" CACHE PATH "Path to LASTools root directory" FORCE)
    else()
        # 默认路径：按你的 PCL 1.14.0 目录先给一个常见位置
        set(LASTOOLS_ROOT "D:/PCL 1.12.1/3rdParty/LAStools" CACHE PATH "Path to LASTools root directory" FORCE)
    endif()
endif()

# 查找LASlib头文件
find_path(LASLIB_INCLUDE_DIR
    NAMES lasreader.hpp
    PATHS 
        "${LASTOOLS_ROOT}/LASlib/inc"
        "$ENV{LASTOOLS_ROOT}/LASlib/inc"
    DOC "LASlib include directory"
)

# 查找LASzip头文件
find_path(LASZIP_INCLUDE_DIR
    NAMES laszip.hpp
    PATHS 
        "${LASTOOLS_ROOT}/LASzip/src"
        "$ENV{LASTOOLS_ROOT}/LASzip/src"
    DOC "LASzip include directory"
)

# 查找LASlib库文件
find_library(LASLIB_LIBRARY
    NAMES LASlib laslib
    PATHS 
        "${LASTOOLS_ROOT}/LASlib/lib"
        "$ENV{LASTOOLS_ROOT}/LASlib/lib"
    DOC "LASlib library"
)

# 处理查找结果
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LASTools
    FOUND_VAR LASTOOLS_FOUND
    REQUIRED_VARS 
        LASLIB_INCLUDE_DIR
        LASZIP_INCLUDE_DIR
        LASLIB_LIBRARY
)

if(LASTOOLS_FOUND)
    # 设置输出变量
    set(LASTOOLS_INCLUDE_DIRS 
        ${LASLIB_INCLUDE_DIR}
        ${LASZIP_INCLUDE_DIR}
    )
    set(LASTOOLS_LIBRARIES ${LASLIB_LIBRARY})
    
    # 创建导入目标（现代CMake方式）
    if(NOT TARGET LASTools::LASlib)
        add_library(LASTools::LASlib UNKNOWN IMPORTED)
        set_target_properties(LASTools::LASlib PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${LASTOOLS_INCLUDE_DIRS}"
            IMPORTED_LOCATION "${LASLIB_LIBRARY}"
        )
    endif()
    
    # 打印状态信息
    message(STATUS "Found LASTools:")
    message(STATUS "  Includes: ${LASTOOLS_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${LASTOOLS_LIBRARIES}")
endif()

mark_as_advanced(
    LASLIB_INCLUDE_DIR
    LASZIP_INCLUDE_DIR
    LASLIB_LIBRARY
)