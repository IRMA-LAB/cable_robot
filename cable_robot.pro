
HEADERS = \
    $$PWD/inc/robot/components/winch.h \
    $$PWD/inc/types.h \
    $$PWD/inc/robot/components/actuator.h \
    $$PWD/inc/robot/components/pulleys_system.h \
    $$PWD/inc/gui/main_gui.h \
    $$PWD/inc/gui/login_window.h \
    $$PWD/inc/robot/cablerobot.h \
    $$PWD/inc/gui/calibration_dialog.h \
    $$PWD/inc/gui/homing_dialog.h

SOURCES = \
    $$PWD/src/robot/components/winch.cpp \
    $$PWD/src/robot/components/actuator.cpp \
    $$PWD/src/robot/components/pulleys_system.cpp \
    $$PWD/src/gui/main_gui.cpp \
    $$PWD/src/gui/login_window.cpp \
    $$PWD/src/types.cpp \
    $$PWD/src/main.cpp \
    $$PWD/src/robot/cablerobot.cpp \
    $$PWD/src/gui/homing_dialog.cpp \
    $$PWD/src/gui/calibration_dialog.cpp

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
unix:!macx: LIBS += -L$$PWD/lib/grab_common/libgrabec/lib/ -lgrabec
INCLUDEPATH += $$PWD/lib/grab_common/libgrabec \
    lib/grab_common/libgrabec/inc
DEPENDPATH += $$PWD/lib/grab_common/libgrabec
unix:!macx: PRE_TARGETDEPS += $$PWD/lib/grab_common/libgrabec/lib/libgrabec.a

# GRAB Real-time lib
unix:!macx: LIBS += -L$$PWD/lib/grab_common/libgrabrt/lib/ -lgrabrt
INCLUDEPATH += $$PWD/lib/grab_common/libgrabrt \
    lib/grab_common/libgrabrt/inc
DEPENDPATH += $$PWD/lib/grab_common/libgrabrt
unix:!macx: PRE_TARGETDEPS += $$PWD/lib/grab_common/libgrabrt/lib/libgrabrt.a

# EtherCAT lib
INCLUDEPATH += /opt/etherlab/include
DEPENDPATH  += /opt/etherlab/lib/
LIBS        += /opt/etherlab/lib/libethercat.a

# State machine lib
unix:!macx: LIBS += -L$$PWD/lib/state_machine/lib/ -lstate_machine
INCLUDEPATH += $$PWD/lib/state_machine $$PWD/lib/state_machine/inc
DEPENDPATH += $$PWD/lib/state_machine
unix:!macx: PRE_TARGETDEPS += $$PWD/lib/state_machine/lib/libstate_machine.a

# GRAB CDPR lib
unix:!macx: LIBS += -L$$PWD/lib/grab_common/libcdpr/lib/ -lcdpr
INCLUDEPATH += $$PWD/lib/grab_common/libcdpr \
    $$PWD/lib/grab_common/libcdpr/inc \
    $$PWD/lib/grab_common/libcdpr/tools
DEPENDPATH += $$PWD/lib/grab_common/libcdpr
unix:!macx: PRE_TARGETDEPS += $$PWD/lib/grab_common/libcdpr/lib/libcdpr.a

# Geometric lib
unix:!macx: LIBS += -L$$PWD/lib/grab_common/libgeom/lib/ -lgeom
INCLUDEPATH += $$PWD/lib/grab_common/libgeom $$PWD/lib/grab_common/libgeom/inc/
DEPENDPATH += $$PWD/lib/grab_common/libgeom
unix:!macx: PRE_TARGETDEPS += $$PWD/lib/grab_common/libgeom/lib/libgeom.a

# Numeric lib
unix:!macx: LIBS += -L$$PWD/lib/grab_common/libnumeric/lib/ -lnumeric
INCLUDEPATH += $$PWD/lib/grab_common/libnumeric \
    $$PWD/lib/grab_common/libnumeric/inc/
DEPENDPATH += $$PWD/lib/grab_common/libnumeric
unix:!macx: PRE_TARGETDEPS += $$PWD/lib/grab_common/libnumeric/lib/libnumeric.a

FORMS += \
    $$PWD/gui/sub_win/actuatorinterface.ui \
    $$PWD/gui/sub_win/actuatorpvtinterface33.ui \
    $$PWD/gui/sub_win/calibrationinterface.ui \
    $$PWD/gui/sub_win/demointerface33.ui \
    $$PWD/gui/sub_win/demointerface66.ui \
    $$PWD/gui/sub_win/hominginterface.ui \
    $$PWD/gui/sub_win/manualinterface33.ui \
    $$PWD/gui/sub_win/manualinterface66.ui \
    $$PWD/gui/cablerobotinterface.ui \
    $$PWD/widgets/main_gui.ui \
    $$PWD/widgets/login_window.ui \
    $$PWD/widgets/calib/calibration_dialog.ui \
    $$PWD/widgets/homing/homing_dialog.ui \
    $$PWD/widgets/homing/homing_interface.ui

RESOURCES += \
    resources/resources.qrc

DEFINES += SRCDIR=\\\"$$PWD/\\\"
