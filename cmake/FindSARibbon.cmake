# cmake/FindSARibbon.cmake
# Find SARibbon library
#
# This module defines:
#  SARIBBON_FOUND - true if SARibbon is found
#  SARIBBON_INCLUDE_DIRS - SARibbon include directories
#  SARIBBON_LIBRARIES - SARibbon libraries to link against

# 设置SARibbon的根目录
if(NOT SARIBBON_ROOT)
set(SARIBBON_ROOT "${CMAKE_SOURCE_DIR}/lib/SARibbon")
endif()

find_path( 
    SARIBBON_INCLUDE_DIR
    NAMES SARibbonBar/SARibbonBar.h
    PATHS 
        "${SARIBBON_ROOT}/src"
    DOC "SARibbon include directory"
)

find_path( 
    QWKCORE_INCLUDE_DIR
    NAMES QWKCore/qwkconfig.h
    PATHS 
        "${SARIBBON_ROOT}/bin_qt5.15.2_MSVC_x64/include/QWindowKit"
    DOC "QWindowKit include directory"
)

find_path( 
    QWKWidgets_INCLUDE_DIR
    NAMES QWKWidgets/qwkwidgetsglobal.h
    PATHS 
        "${SARIBBON_ROOT}/bin_qt5.15.2_MSVC_x64/include/QWindowKit"
    DOC "QWindowKit include directory"
)


find_library(
    SARIBBON_LIBRARY
    NAMES SARibbonBar
    PATHS 
        "${SARIBBON_ROOT}/lib"
    DOC "SARibbon library"
)

find_library(
    QWKCORE_LIBRARY
    NAMES QWKCore 
    PATHS 
        "${SARIBBON_ROOT}/bin_qt5.15.2_MSVC_x64/lib"
    DOC "QWindowKit library"
)

find_library(
    QWKWidgets_LIBRARY
    NAMES QWKWidgets 
    PATHS 
        "${SARIBBON_ROOT}/bin_qt5.15.2_MSVC_x64/lib"
    DOC "QWindowKit library"
)

# 处理查找结果
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SARibbon
    FOUND_VAR SARIBBON_FOUND
    REQUIRED_VARS 
        SARIBBON_INCLUDE_DIR
        SARIBBON_LIBRARY
        QWKCORE_INCLUDE_DIR
        QWKCORE_LIBRARY
        QWKWidgets_INCLUDE_DIR
        QWKWidgets_LIBRARY
)

if(SARIBBON_FOUND)
    # 设置输出变量
    set(SARIBBON_INCLUDE_DIRS 
        ${SARIBBON_INCLUDE_DIR}
        ${QWKCORE_INCLUDE_DIR}
        ${QWKWidgets_INCLUDE_DIR}
    )

    set(SARIBBON_LIBRARIES ${SARIBBON_LIBRARY} ${QWKCORE_LIBRARY} ${QWKWidgets_LIBRARY})
    
    # 创建导入目标（现代CMake方式）
    if(NOT TARGET SARibbon::SARibbon)
        add_library(SARibbon::SARibbon UNKNOWN IMPORTED)
        set_target_properties(SARibbon::SARibbon PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${SARIBBON_INCLUDE_DIRS}"
            IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
            IMPORTED_LOCATION "${SARIBBON_LIBRARY}"
            IMPORTED_LOCATION_DEBUG "${SARIBBON_LIBRARY}"
            IMPORTED_LOCATION_RELEASE "${SARIBBON_LIBRARY}"
        )

        set_property(TARGET SARibbon::SARibbon APPEND PROPERTY
            INTERFACE_LINK_LIBRARIES 
             "${QWKCORE_LIBRARY}"
             "${QWKWidgets_LIBRARY}"
        )
    endif()
    
    # 打印状态信息
    message(STATUS "Found SARibbon:")
    message(STATUS "  Includes: ${SARIBBON_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${SARIBBON_LIBRARIES}")
endif()

mark_as_advanced(
    SARIBBON_INCLUDE_DIR
    SARIBBON_LIBRARY
)
