
HEADERS += inc/threads.h

SOURCES += test/libgrabrt_test.cpp \
      src/threads.cpp

INCLUDEPATH += inc \
      ../libnumeric/inc

QT       += testlib
QT       -= gui

TARGET = libgrabrt_test

CONFIG   += console c++11
CONFIG   -= app_bundle

TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
