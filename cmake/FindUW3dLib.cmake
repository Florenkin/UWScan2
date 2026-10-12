# cmake/FindUW3dLib.cmake
# Find UW3dLib library
#
# This module defines:
#  UW3DLIB_FOUND - true if UW3dLib is found
#  UW3DLIB_INCLUDE_DIRS - UW3dLib include directories
#  UW3DLIB_LIBRARIES - UW3dLib libraries to link against

# 设置UW3dLib的根目录
if(NOT UW3DLIB_ROOT)
set(UW3DLIB_ROOT "${CMAKE_SOURCE_DIR}/lib/UW3dLib")
endif()

find_path( 
    UW3DLIB_INCLUDE_DIR
    NAMES LinkHouAPI.h
    PATHS 
        "${UW3DLIB_ROOT}/include"
    DOC "UW3dLib include directory"
)

set(UW3DLIB_LIB_NAMES
    LinkHouAPI
    TailPlc
    DualRobotCalirationLib
    UW3DReconstruction_DLL
    vtkGUISupportQt-9.1
    vtkRenderingOpenGL2-9.1
    vtkRenderingQt-9.1
    libboost_thread-vc142-mt-x64-1_78
    )

set(UW3DLIB_LIBRARY)
foreach(LIB_NAME ${UW3DLIB_LIB_NAMES})
    find_library(
        ${LIB_NAME}_LIBRARY
        NAMES ${LIB_NAME}
        PATHS 
            "${UW3DLIB_ROOT}/lib"
        NO_DEFAULT_PATH
        DOC "UW3dLib library ${LIB_NAME}"
    )
    if(${LIB_NAME}_LIBRARY)
        list(APPEND UW3DLIB_LIBRARY ${${LIB_NAME}_LIBRARY})
    endif()
endforeach()


# 处理查找结果
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(UW3DLIB
    FOUND_VAR UW3DLIB_FOUND
    REQUIRED_VARS 
        UW3DLIB_INCLUDE_DIR
        UW3DLIB_LIBRARY
)

if(UW3DLIB_FOUND)
    # 设置输出变量
    set(UW3DLIB_INCLUDE_DIRS 
        ${UW3DLIB_INCLUDE_DIR}
    )

    set(UW3DLIB_LIBRARIES ${UW3DLIB_LIBRARY})
    
    # 创建导入目标（现代CMake方式）
    if(NOT TARGET UW3DLIB::UW3DLIB)
        add_library(UW3DLIB::UW3DLIB INTERFACE IMPORTED)  # 使用 INTERFACE 而不是 UNKNOWN
        
        # 设置包含目录
        set_target_properties(UW3DLIB::UW3DLIB PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${UW3DLIB_INCLUDE_DIRS}"
        )
        
        # 分别链接每个库文件
        foreach(lib ${UW3DLIB_LIBRARIES})
            # 为每个库创建单独的导入目标
            get_filename_component(lib_name "${lib}" NAME_WE)
            if(NOT TARGET UW3DLIB::${lib_name})
                add_library(UW3DLIB::${lib_name} UNKNOWN IMPORTED)
                set_target_properties(UW3DLIB::${lib_name} PROPERTIES
                    IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
                    IMPORTED_LOCATION "${lib}"
                    IMPORTED_LOCATION_DEBUG "${lib}"
                    IMPORTED_LOCATION_RELEASE "${lib}"
                )
            endif()
            # 将每个库链接到主目标
            target_link_libraries(UW3DLIB::UW3DLIB INTERFACE UW3DLIB::${lib_name})
        endforeach()
    endif()
    
    # 打印状态信息
    message(STATUS "Found UW3DLIB:")
    message(STATUS "  Includes: ${UW3DLIB_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${UW3DLIB_LIBRARIES}")
endif()

mark_as_advanced(  
    UW3DLIB_FOUND
    UW3DLIB_INCLUDE_DIRS
    UW3DLIB_LIBRARIES
)
