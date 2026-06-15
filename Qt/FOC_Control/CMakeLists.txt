cmake_minimum_required(VERSION 3.19)
project(FOC_Control LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets Charts)

qt_standard_project_setup()

qt_add_executable(FOC_Control
    WIN32 MACOSX_BUNDLE
    main.cpp
    serial_backend.cpp
    serial_backend.h
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

target_link_libraries(FOC_Control
    PRIVATE
        Qt::Core
        Qt::Widgets
        Qt::Charts
)

if (WIN32)
    target_link_libraries(FOC_Control PRIVATE setupapi)
endif()

install(TARGETS FOC_Control
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
