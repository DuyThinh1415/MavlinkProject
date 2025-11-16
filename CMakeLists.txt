cmake_minimum_required(VERSION 3.16)

project(MavlinkConnectProject VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)



list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/mavlink/install")

find_package(Qt6 REQUIRED COMPONENTS Core Quick Network  HttpServer)
find_package(Qt6 REQUIRED COMPONENTS Core)

find_package(MAVLink REQUIRED)

qt_standard_project_setup(REQUIRES 6.7)

qt_add_executable(appMavlinkConnectProject
    main.cpp
)

qt_add_qml_module(appMavlinkConnectProject
    URI MavlinkConnectProject
    VERSION 1.0
    QML_FILES Main.qml
    SOURCES CommManager/maincomm.h CommManager/maincomm.cpp
    SOURCES CommManager/mavlinkcommmanager.h CommManager/mavlinkcommmanager.cpp
    SOURCES CommManager/ParseXML/parsexml.h CommManager/ParseXML/parsexml.cpp
    SOURCES CommManager/ParseXML/httphost.h CommManager/ParseXML/httphost.cpp
    SOURCES CommManager/ackhandler.h CommManager/ackhandler.cpp
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appMavlinkConnectProject PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appMavlinkConnectProject
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appMavlinkConnectProject
    PRIVATE Qt6::Quick
)
target_link_libraries(appMavlinkConnectProject PRIVATE
    Qt6::Core
    MAVLink::mavlink
    Qt6::Network
    Qt6::HttpServer)
target_link_libraries(appMavlinkConnectProject PRIVATE Qt6::Core)

include(GNUInstallDirs)
install(TARGETS appMavlinkConnectProject
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
