cmake_minimum_required(VERSION 3.16)

project(gui VERSION 0.1 LANGUAGES CXX)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
find_package(Qt6 REQUIRED COMPONENTS Quick)

qt_standard_project_setup(REQUIRES 6.10)

qt_add_executable(appgui
    main.cpp
)

qt_add_qml_module(appgui
    URI gui
    QML_FILES
        Main.qml
)

set_target_properties(appgui PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER com.soundlink.receiver
    MACOSX_BUNDLE_BUNDLE_NAME "SoundLink"
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(IOS)
    target_sources(appgui PRIVATE assets/AppIcons.xcassets)
    set_source_files_properties(assets/AppIcons.xcassets PROPERTIES
        MACOSX_PACKAGE_LOCATION Resources)
    set_target_properties(appgui PROPERTIES
        XCODE_ATTRIBUTE_ASSETCATALOG_COMPILER_APPICON_NAME AppIcon
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info-ios.plist.in"
        XCODE_ATTRIBUTE_CODE_SIGN_STYLE Automatic
        XCODE_GENERATE_SCHEME TRUE
        XCODE_SCHEME_LAUNCH_CONFIGURATION Release)
elseif(APPLE)
    set_target_properties(appgui PROPERTIES
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in")
endif()
if(ANDROID)
    qt_add_android_permission(appgui NAME android.permission.RECORD_AUDIO)
endif()

target_link_libraries(appgui
    PRIVATE Qt6::Quick
)

install(TARGETS appgui
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

