#-------------------------------------------------
#
# Project created by QtCreator 2017-01-30T16:02:03
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RobotControl
TEMPLATE = app
CONFIG += c++11


SOURCES += main.cpp\
        mainwindow.cpp \
    socket.cpp \
    playfield.cpp \
    robotfinder.cpp \
    controller.cpp

HEADERS  += mainwindow.h \
    socket.h \
    playfield.h \
    defines.h \
    robotfinder.h \
    controller.h

LIBS += "/usr/local/lib/libfuzzylite-static.a"
