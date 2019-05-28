
HEADERS = \
    $$PWD/inc/robot/cablerobot.h \
    $$PWD/inc/robot/components/actuator.h \
    $$PWD/inc/robot/components/winch.h \
    $$PWD/inc/robot/components/pulleys_system.h \
    $$PWD/inc/gui/main_gui.h \
    $$PWD/inc/gui/login_window.h \
    $$PWD/inc/gui/calib/calibration_dialog.h \
    $$PWD/inc/gui/homing/homing_dialog.h \
    $$PWD/inc/gui/homing/homing_interface.h \
    $$PWD/inc/gui/homing/homing_interface_proprioceptive.h \
    $$PWD/inc/gui/homing/init_torque_form.h \
    $$PWD/inc/gui/apps/joints_pvt_dialog.h \
    $$PWD/inc/gui/apps/my3dscatterwidget.h \
    $$PWD/inc/gui/apps/chartview.h \
    $$PWD/inc/gui/apps/input_form.h \
    $$PWD/inc/homing/homing_proprioceptive.h \
    $$PWD/inc/homing/matlab_thread.h \
    $$PWD/inc/ctrl/controller_base.h \
    $$PWD/inc/ctrl/controller_singledrive.h \
    $$PWD/inc/ctrl/controller_joints_pvt.h \
    $$PWD/inc/ctrl/winch_torque_controller.h \
    $$PWD/inc/utils/types.h \
    $$PWD/inc/utils/macros.h \
    $$PWD/inc/utils/msgs.h \
    $$PWD/inc/utils/easylog_wrapper.h \
    $$PWD/inc/state_estimation/ext_kalman_filter.h \
    $$PWD/inc/debug/single_drive_sysid.h \
    $$PWD/inc/apps/joints_pvt_app.h \
    $$PWD/lib/easyloggingpp/src/easylogging++.h \
    $$PWD/lib/grab_common/grabcommon.h \
    $$PWD/lib/grab_common/pid/pid.h \
    $$PWD/inc/gui/apps/manual_control_dialog.h \
    $$PWD/inc/apps/manual_control_app.h

SOURCES = \
    $$PWD/src/main.cpp \
    $$PWD/src/robot/cablerobot.cpp \
    $$PWD/src/robot/components/actuator.cpp \
    $$PWD/src/robot/components/winch.cpp \
    $$PWD/src/robot/components/pulleys_system.cpp \
    $$PWD/src/gui/main_gui.cpp \
    $$PWD/src/gui/login_window.cpp \
    $$PWD/src/gui/calib/calibration_dialog.cpp \
    $$PWD/src/gui/homing/homing_dialog.cpp \
    $$PWD/src/gui/homing/homing_interface.cpp \
    $$PWD/src/gui/homing/homing_interface_proprioceptive.cpp \
    $$PWD/src/gui/homing/init_torque_form.cpp \
    $$PWD/src/gui/apps/joints_pvt_dialog.cpp \
    $$PWD/src/gui/apps/my3dscatterwidget.cpp \
    $$PWD/src/gui/apps/chartview.cpp \
    $$PWD/src/gui/apps/input_form.cpp \
    $$PWD/src/homing/homing_proprioceptive.cpp \
    $$PWD/src/homing/matlab_thread.cpp \
    $$PWD/src/ctrl/controller_base.cpp \
    $$PWD/src/ctrl/controller_singledrive.cpp \
    $$PWD/src/ctrl/controller_joints_pvt.cpp \
    $$PWD/src/ctrl/winch_torque_controller.cpp \
    $$PWD/src/utils/msgs.cpp \
    $$PWD/src/utils/easylog_wrapper.cpp \
    $$PWD/src/state_estimation/ext_kalman_filter.cpp \
    $$PWD/src/debug/single_drive_sysid.cpp \
    $$PWD/src/apps/joints_pvt_app.cpp \
    $$PWD/lib/easyloggingpp/src/easylogging++.cc \
    $$PWD/lib/grab_common/grabcommon.cpp \
    $$PWD/lib/grab_common/pid/pid.cpp \
    $$PWD/src/gui/apps/manual_control_dialog.cpp \
    $$PWD/src/apps/manual_control_app.cpp

INCLUDEPATH += \
    $$PWD/inc \
    $$PWD/lib/grab_common \
    $$PWD/lib/easyloggingpp/src


QT += core gui widgets datavisualization charts

CONFIG += c++11 console
CONFIG -= app_bundle

TEMPLATE = app

TARGET = CableRobotApp

DEFINES += ELPP_QT_LOGGING    \
          ELPP_STL_LOGGING   \
          ELPP_MULTI_LOGGER_SUPPORT \
          ELPP_THREAD_SAFE \
          ELPP_FRESH_LOG_FILE

DEFINES += SRCDIR=\\\"$$PWD/\\\"

# DEBUG
#HEADERS += \
#    $$PWD/lib/grab_common/libgrabec/inc/ethercatmaster.h \
#    $$PWD/lib/grab_common/libgrabec/inc/ethercatslave.h \
#    $$PWD/lib/grab_common/libgrabec/inc/types.h \
#    $$PWD/lib/grab_common/libgrabec/inc/slaves/goldsolowhistledrive.h
#SOURCES += \
#    $$PWD/lib/grab_common/libgrabec/src/ethercatmaster.cpp \
#    $$PWD/lib/grab_common/libgrabec/src/ethercatslave.cpp \
#    $$PWD/lib/grab_common/libgrabec/src/slaves/goldsolowhistledrive.cpp
#INCLUDEPATH += $$PWD/lib/grab_common/libgrabec/inc

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
    $$PWD/widgets/main_gui.ui \
    $$PWD/widgets/login_window.ui \
    $$PWD/widgets/calib/calibration_dialog.ui \
    $$PWD/widgets/homing/homing_dialog.ui \
    $$PWD/widgets/homing/homing_interface_proprioceptive.ui\
    $$PWD/widgets/homing/init_torque_form.ui \
    $$PWD/widgets/apps/joints_pvt_dialog.ui \
    $$PWD/widgets/apps/my3dscatterwidget.ui \
    $$PWD/widgets/apps/input_form.ui \
    $$PWD/widgets/apps/manual_control_dialog.ui

RESOURCES += \
    resources/resources.qrc
