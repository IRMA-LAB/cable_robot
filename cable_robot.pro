
HEADERS = \
    inc/servomotor.h \
    inc/winch.h \
    inc/cablerobotmaster.h \
    inc/cablerobot.h \
    inc/types.h

SOURCES = \
    src/servomotor.cpp \
    src/winch.cpp \
    src/cablerobotmaster.cpp \
    src/cablerobot.cpp

INCLUDEPATH += \
    inc \
    lib/grab_common \
    lib/grab_common/libgrabec/inc \
    lib/grab_common/libgrabrt/inc \
    /opt/etherlab/include/

QT += core gui widgets

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# GRAB Ethercat lib
unix:!macx: LIBS += -L$$PWD/lib/grab_common/libgrabec/build/ -lgrabec
INCLUDEPATH += $$PWD/lib/grab_common/libgrabec/build
DEPENDPATH += $$PWD/lib/grab_common/libgrabec/build
unix:!macx: PRE_TARGETDEPS += $$PWD/lib/grab_common/libgrabec/build/libgrabec.a
