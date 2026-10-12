cmake_minimum_required(VERSION 3.16)

project(gui VERSION 0.1 LANGUAGES CXX)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
if(IOS)
    enable_language(OBJCXX)
    set(CMAKE_OBJCXX_STANDARD 17)
    set(CMAKE_OBJCXX_STANDARD_REQUIRED ON)
    set_source_files_properties(receiver.cpp PROPERTIES LANGUAGE OBJCXX)
endif()
find_package(Threads REQUIRED)

find_package(Qt6 REQUIRED COMPONENTS Quick)

qt_standard_project_setup(REQUIRES 6.10)

qt_add_executable(appgui
    main.cpp
    receiver.cpp
    receiver.h
    transmitter.h
    transmitter.inc
    receive_rate.h
    snr_estimate.h
    ../src/framework.h
    ../src/generic.h
    ../src/bch.h
    ../src/perm.h
    ../src/viterbi.h
    ../src/puncture.h
    ../src/math.h
    ../src/filter.h
    ../src/physical_layer.h
    ../src/dsp.h
    ../src/miniaudio.h
    ../src/device.h
)

qt_add_qml_module(appgui
    URI gui
    QML_FILES
        Main.qml
        TransmitterPanel.qml
        SnrPlot.qml
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appgui PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER com.soundlink.receiver
    MACOSX_BUNDLE_BUNDLE_NAME "SoundLink Receiver"
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
    target_link_libraries(appgui PRIVATE "-framework AVFoundation")
elseif(APPLE)
    set_target_properties(appgui PROPERTIES
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in")
endif()
if(ANDROID)
    qt_add_android_permission(appgui NAME android.permission.RECORD_AUDIO)
endif()

target_link_libraries(appgui
    PRIVATE Qt6::Quick Threads::Threads ${CMAKE_DL_LIBS}
)

install(TARGETS appgui
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

