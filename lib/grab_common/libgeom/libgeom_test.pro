#-------------------------------------------------
#
# Project created by QtCreator 2018-08-24T17:04:34
#
#-------------------------------------------------

QT       += testlib

QT       -= gui

TARGET = libgeom_test
CONFIG   += console c++11
CONFIG   -= app_bundle

TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    inc/rotations.h

SOURCES += \
        test/libgeom_test.cpp \
        src/rotations.cpp

INCLUDEPATH = \
        inc \
        ../libnumeric/inc \
        ../libnumeric/src

DEFINES += SRCDIR=\\\"$$PWD/\\\"
