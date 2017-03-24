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

DEFINES += FL_CPP11

SOURCES += main.cpp\
        mainwindow.cpp \
    socket.cpp \
    playfield.cpp \
    robotfinder.cpp \
    controller.cpp \
    neuralnet.cpp \
    neuron.cpp

HEADERS  += mainwindow.h \
    socket.h \
    playfield.h \
    defines.h \
    robotfinder.h \
    controller.h \
    neuralnet.h \
    neuron.h

LIBS += "/usr/local/lib/libfuzzylite-static.a"
