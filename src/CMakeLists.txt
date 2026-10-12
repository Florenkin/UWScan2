set(UWSCAN_PUBLIC_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(UWSCAN_UI_DIR "${PROJECT_SOURCE_DIR}/resources/ui")

file(GLOB UWSCAN_SETTINGS_HEADERS "${UWSCAN_PUBLIC_INCLUDE_DIR}/setting/*.h" "${UWSCAN_PUBLIC_INCLUDE_DIR}/setting/*.hpp")
file(GLOB UWSCAN_POINTCLOUD_HEADERS "${UWSCAN_PUBLIC_INCLUDE_DIR}/pointcloud/*.h" "${UWSCAN_PUBLIC_INCLUDE_DIR}/pointcloud/*.hpp")
file(GLOB UWSCAN_SCANNER_HEADERS "${UWSCAN_PUBLIC_INCLUDE_DIR}/scanner/*.h" "${UWSCAN_PUBLIC_INCLUDE_DIR}/scanner/*.hpp")
file(GLOB UWSCAN_SERVER_HEADERS "${UWSCAN_PUBLIC_INCLUDE_DIR}/server/*.h" "${UWSCAN_PUBLIC_INCLUDE_DIR}/server/*.hpp")
file(GLOB UWSCAN_MAINWINDOW_HEADERS "${UWSCAN_PUBLIC_INCLUDE_DIR}/mainwindow/*.h" "${UWSCAN_PUBLIC_INCLUDE_DIR}/mainwindow/*.hpp")

function(uwscan_add_module target)
    add_library(${target} STATIC ${ARGN})
    add_library(UWScan::${target} ALIAS ${target})
    # Public headers include AUTOUIC-generated headers, so dependent targets
    # must also see this module's binary directory during the build.
    target_include_directories(${target} PUBLIC
        "$<BUILD_INTERFACE:${UWSCAN_PUBLIC_INCLUDE_DIR}>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/${target}_autogen/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/${target}_autogen/include_$<CONFIG>>")
    set_target_properties(${target} PROPERTIES AUTOMOC ON AUTOUIC ON AUTOUIC_SEARCH_PATHS "${UWSCAN_UI_DIR}" POSITION_INDEPENDENT_CODE ON)
endfunction()

uwscan_add_module(uwscan_settings setting/settingsmanager.cpp setting/settingwidget.cpp ${UWSCAN_SETTINGS_HEADERS} "${PROJECT_SOURCE_DIR}/resources/ui/settingwidget.ui")
target_link_libraries(uwscan_settings PUBLIC Qt5::Core Qt5::Widgets)

uwscan_add_module(uwscan_widgets imagewidget.cpp messagestation.cpp infofullstatusbar.cpp
    "${UWSCAN_PUBLIC_INCLUDE_DIR}/imagewidget.h" "${UWSCAN_PUBLIC_INCLUDE_DIR}/messagestation.h"
    "${UWSCAN_PUBLIC_INCLUDE_DIR}/infofullstatusbar.h" "${PROJECT_SOURCE_DIR}/resources/ui/messagestation.ui")
target_include_directories(uwscan_widgets PUBLIC "${BASLER_PYLON_INCLUDE_DIR}")
target_link_libraries(uwscan_widgets PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets ${OpenCV_LIBRARIES})

uwscan_add_module(uwscan_pointcloud pointcloud/cloudtreeitem.cpp pointcloud/cloudtreewidget.cpp pointcloud/cloudwindow.cpp pointcloud/filterdialog.cpp pointcloud/importoptiondialog.cpp ${UWSCAN_POINTCLOUD_HEADERS})
target_link_libraries(uwscan_pointcloud PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets ${PCL_LIBRARIES})

uwscan_add_module(uwscan_scanner
    scanner/ImgProcess.cpp scanner/QuadricSurfaceFitting.cpp scanner/QuadricSurfaceFitting2.cpp
    scanner/associatedpointsmanager.cpp scanner/camera.cpp scanner/imagethread.cpp scanner/modbusmotor.cpp
    scanner/reconstructor.cpp scanner/scanner.cpp scanner/scannerinterface.cpp scanner/tcpserver.cpp
    scanner/uwscan3d_algorithms.cpp
    ${UWSCAN_SCANNER_HEADERS}
    "${PROJECT_SOURCE_DIR}/resources/ui/scannerinterface.ui")
target_include_directories(uwscan_scanner PUBLIC "${BASLER_PYLON_INCLUDE_DIR}")
target_link_directories(uwscan_scanner PUBLIC "${BASLER_PYLON_LIB_DIR}")
target_link_libraries(uwscan_scanner PUBLIC uwscan_settings uwscan_widgets uwscan_pointcloud Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Network Qt5::SerialPort Qt5::SerialBus ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} Ceres::Ceres UW3DLIB::UW3DLIB)

uwscan_add_module(uwscan_server server/commandserver.cpp server/commanddialog.cpp ${UWSCAN_SERVER_HEADERS})
target_link_libraries(uwscan_server PUBLIC Qt5::Core Qt5::Network uwscan_scanner)

uwscan_add_module(uwscan_app_shell mainwindow/SARibbon.cpp mainwindow/advancedstatusbar.cpp mainwindow/uwscanmainwindow.cpp ${UWSCAN_MAINWINDOW_HEADERS})
target_link_libraries(uwscan_app_shell PUBLIC uwscan_settings uwscan_widgets uwscan_pointcloud uwscan_scanner uwscan_server Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick Qt5::Widgets Qt5::Network ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} SARibbon::SARibbon)

add_executable(${PROJECT_NAME} WIN32 main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE "${UWSCAN_PUBLIC_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PRIVATE uwscan_app_shell)

set(RESOURCES_BASE_DIR "${CMAKE_SOURCE_DIR}/resources")
qt5_add_resources(ICON_RESOURCE "${RESOURCES_BASE_DIR}/images/Icon.qrc")
qt5_add_resources(QRIBBON_RESOURCE "${RESOURCES_BASE_DIR}/QRibbon.qrc")
target_sources(${PROJECT_NAME} PRIVATE ${ICON_RESOURCE} ${QRIBBON_RESOURCE})

if(WIN32)
    set(QT_PLUGINS_DIR "${QT_ROOT_DIR}/plugins")
    set(QT_PLATFORMS_DIR "${QT_PLUGINS_DIR}/platforms")
    set(APP_RUNTIME_DLLS
        "${PROJECT_SOURCE_DIR}/lib/ceresLib/ceres/bin/ceres.dll" "${PROJECT_SOURCE_DIR}/lib/ceresLib/gflags/bin/gflags.dll"
        "${PROJECT_SOURCE_DIR}/lib/ceresLib/glog/bin/glog.dll" "${PROJECT_SOURCE_DIR}/lib/UW3dLib/lib/DualRobotCalibrationLib.dll"
        "${PROJECT_SOURCE_DIR}/lib/UW3dLib/lib/LinkHouAPI.dll" "${PROJECT_SOURCE_DIR}/lib/UW3dLib/lib/TailPlc.dll"
        "${PROJECT_SOURCE_DIR}/lib/UW3dLib/lib/UW3DReconstruction_DLL.dll" "${PROJECT_SOURCE_DIR}/lib/SARibbon/lib/SARibbonBar.dll"
        "${PROJECT_SOURCE_DIR}/lib/SARibbon/bin_qt5.15.2_MSVC_x64/bin/QWKCore.dll" "${PROJECT_SOURCE_DIR}/lib/SARibbon/bin_qt5.15.2_MSVC_x64/bin/QWKWidgets.dll")
    if(QT_WINDEPLOYQT_EXECUTABLE)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND "${QT_WINDEPLOYQT_EXECUTABLE}" --dir "${PROJECT_SOURCE_DIR}/bin" --no-translations "$<TARGET_FILE:${PROJECT_NAME}>" COMMAND ${CMAKE_COMMAND} -E copy_if_different ${APP_RUNTIME_DLLS} "${PROJECT_SOURCE_DIR}/bin" COMMENT "Deploying Qt runtime and plugins with windeployqt" VERBATIM)
    elseif(EXISTS "${QT_PLATFORMS_DIR}/qwindows.dll")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E make_directory "${PROJECT_SOURCE_DIR}/bin/platforms" COMMAND ${CMAKE_COMMAND} -E copy_if_different "${QT_PLATFORMS_DIR}/qwindows.dll" "${PROJECT_SOURCE_DIR}/bin/platforms/qwindows.dll" COMMAND ${CMAKE_COMMAND} -E copy_if_different ${APP_RUNTIME_DLLS} "${PROJECT_SOURCE_DIR}/bin" COMMENT "Copying Qt Windows platform plugin" VERBATIM)
    else()
        message(WARNING "Qt platform plugin deployment is not configured. qwindows.dll was not found.")
    endif()
endif()
