
HEADERS = \
    $$PWD/inc/robot/components/winch.h \
    $$PWD/inc/types.h \
    $$PWD/inc/robot/components/actuator.h \
    $$PWD/inc/robot/components/pulleys_system.h \
    $$PWD/inc/gui/main_gui.h \
    $$PWD/inc/gui/login_window.h \
    $$PWD/inc/robot/cablerobot.h \
    inc/gui/calibration_dialog.h \
    inc/gui/homing_dialog.h

SOURCES = \
    $$PWD/src/robot/components/winch.cpp \
    $$PWD/src/robot/components/actuator.cpp \
    $$PWD/src/robot/components/pulleys_system.cpp \
    $$PWD/src/gui/main_gui.cpp \
    $$PWD/src/gui/login_window.cpp \
    $$PWD/src/types.cpp \
    $$PWD/src/main.cpp \
    $$PWD/src/robot/cablerobot.cpp \
    src/gui/homing_dialog.cpp \
    src/gui/calibration_dialog.cpp

INCLUDEPATH += \
    $$PWD/inc \
    $$PWD/lib/grab_common


QT += core gui widgets

CONFIG += c++11 console
CONFIG -= app_bundle

TEMPLATE = app

TARGET = CableRobotApp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# GRAB Ethercat lib
unix:!macx: LIBS += -L$$PWD/lib/grab_common/libgrabec/build/ -lgrabec
INCLUDEPATH += $$PWD/lib/grab_common/libgrabec/build \
    lib/grab_common/libgrabec/inc
DEPENDPATH += $$PWD/lib/grab_common/libgrabec/build
unix:!macx: PRE_TARGETDEPS += $$PWD/lib/grab_common/libgrabec/build/libgrabec.a

# GRAB Real-time lib
unix:!macx: LIBS += -L$$PWD/lib/grab_common/libgrabrt/build/ -lgrabrt
INCLUDEPATH += $$PWD/lib/grab_common/libgrabrt/build \
    lib/grab_common/libgrabrt/inc
DEPENDPATH += $$PWD/lib/grab_common/libgrabrt/build
unix:!macx: PRE_TARGETDEPS += $$PWD/lib/grab_common/libgrabrt/build/libgrabrt.a

# EtherCAT lib
INCLUDEPATH += /opt/etherlab/include
DEPENDPATH  += /opt/etherlab/lib/
LIBS        += /opt/etherlab/lib/libethercat.a

# State machine lib
unix:!macx: LIBS += -L$$PWD/lib/grab_common/state_machine/lib/ -lstate_machine
INCLUDEPATH += $$PWD/lib/grab_common/state_machine
DEPENDPATH += $$PWD/lib/grab_common/state_machine
unix:!macx: PRE_TARGETDEPS += $$PWD/lib/grab_common/state_machine/lib/libstate_machine.a

FORMS += \
    gui/sub_win/actuatorinterface.ui \
    gui/sub_win/actuatorpvtinterface33.ui \
    gui/sub_win/calibrationinterface.ui \
    gui/sub_win/demointerface33.ui \
    gui/sub_win/demointerface66.ui \
    gui/sub_win/hominginterface.ui \
    gui/sub_win/manualinterface33.ui \
    gui/sub_win/manualinterface66.ui \
    gui/cablerobotinterface.ui \
    widgets/main_gui.ui \
    widgets/login_window.ui \
    widgets/calibration_dialog.ui \
    widgets/homing_dialog.ui

RESOURCES += \
    resources/resources.qrc
