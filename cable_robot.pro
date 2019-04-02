
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
    $$PWD/inc/gui/homing/homing_interface_vision.h \
    $$PWD/inc/gui/homing/homing_interface_proprioceptive.h \
    $$PWD/inc/gui/homing/init_torque_form.h \
    $$PWD/inc/gui/homing/camera_widget.h \
    $$PWD/inc/gui/homing/camera_calib_dialog.h \
    $$PWD/inc/gui/homing/camera_calib_settings_dialog.h \
    $$PWD/inc/gui/homing/camera_calib_app.h \
    $$PWD/inc/homing/homing_proprioceptive.h \
    $$PWD/inc/homing/homing_vision_app.h \
    $$PWD/inc/homing/matlab_thread.h \
    $$PWD/inc/ctrl/controller_base.h \
    $$PWD/inc/ctrl/controller_singledrive.h \
    $$PWD/inc/utils/types.h \
    $$PWD/inc/utils/macros.h \
    $$PWD/inc/utils/msgs.h \
    $$PWD/inc/utils/easylog_wrapper.h \
    $$PWD/lib/easyloggingpp/src/easylogging++.h \
    $$PWD/lib/grab_common/grabcommon.h \
    $$PWD/lib/grab_common/bitfield.h \
    $$PWD/lib/grab_common/pid/pid.h \

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
    $$PWD/src/gui/homing/homing_interface_vision.cpp \
    $$PWD/src/gui/homing/homing_interface_proprioceptive.cpp \
    $$PWD/src/gui/homing/init_torque_form.cpp \
    $$PWD/src/gui/homing/camera_widget.cpp \
    $$PWD/src/gui/homing/camera_calib_dialog.cpp \
    $$PWD/src/gui/homing/camera_calib_settings_dialog.cpp \
    $$PWD/src/gui/homing/camera_calib_app.cpp \
    $$PWD/src/homing/homing_proprioceptive.cpp \
    $$PWD/src/homing/homing_vision_app.cpp \
    $$PWD/src/homing/matlab_thread.cpp \
    $$PWD/src/ctrl/controller_base.cpp \
    $$PWD/src/ctrl/controller_singledrive.cpp \
    $$PWD/src/utils/msgs.cpp \
    $$PWD/src/utils/easylog_wrapper.cpp \
    $$PWD/lib/easyloggingpp/src/easylogging++.cc \
    $$PWD/lib/grab_common/grabcommon.cpp \
    $$PWD/lib/grab_common/pid/pid.cpp \

INCLUDEPATH += \
    $$PWD/inc \
    $$PWD/lib/grab_common \
    $$PWD/lib/easyloggingpp/src

QT += core gui widgets multimedia multimediawidgets

CONFIG += c++11 console static
CONFIG -= app_bundle

TEMPLATE = app

TARGET = CableRobotApp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

DEFINES += ELPP_QT_LOGGING    \
          ELPP_STL_LOGGING   \
          ELPP_MULTI_LOGGER_SUPPORT \
          ELPP_THREAD_SAFE \
          ELPP_FRESH_LOG_FILE

DEFINES += SRCDIR=\\\"$$PWD/\\\"

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

# OpenCV
INCLUDEPATH += /home/simo/opencv-4.0.1/build/include \
               /usr/local/include/opencv4
LIBS += -L"/home/simo/opencv-4.0.1/build/lib"
LIBS += -lopencv_calib3d
LIBS += -lopencv_core
LIBS += -lopencv_features2d
LIBS += -lopencv_flann
LIBS += -lopencv_highgui
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_imgproc
LIBS += -lopencv_ml
LIBS += -lopencv_objdetect
LIBS += -lopencv_photo
LIBS += -lopencv_stitching
LIBS += -lopencv_video
LIBS += -lopencv_videoio

FORMS += \
    $$PWD/widgets/main_gui.ui \
    $$PWD/widgets/login_window.ui \
    $$PWD/widgets/calib/calibration_dialog.ui \
    $$PWD/widgets/homing/homing_dialog.ui \
    $$PWD/widgets/homing/homing_interface_vision.ui \
    $$PWD/widgets/homing/homing_interface_proprioceptive.ui\
    $$PWD/widgets/homing/init_torque_form.ui \
    $$PWD/widgets/homing/camera_widget.ui \
    $$PWD/widgets/homing/camera_calib_dialog.ui \
    $$PWD/widgets/homing/camera_calib_app.ui \
    $$PWD/widgets/homing/camera_calib_settings_dialog.ui

RESOURCES += resources/resources.qrc
