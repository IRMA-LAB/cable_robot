HEADERS = \
    $$PWD/inc/robot/cablerobot.h \
    $$PWD/inc/robot/components/actuator.h \
    $$PWD/inc/robot/components/winch.h \
    $$PWD/inc/robot/components/pulleys_system.h \
    $$PWD/inc/gui/main_gui.h \
    $$PWD/inc/gui/login_window.h \
    $$PWD/inc/gui/calib/calibration_dialog.h \
    $$PWD/inc/gui/calib/calib_interface_excitation.h \
    $$PWD/inc/gui/homing/homing_dialog.h \
    $$PWD/inc/gui/homing/homing_interface.h \
    $$PWD/inc/gui/homing/homing_interface_vision.h \
    $$PWD/inc/gui/homing/homing_interface_proprioceptive.h \
    $$PWD/inc/gui/homing/init_torque_form.h \
    $$PWD/inc/gui/apps/joints_pvt_dialog.h \
    $$PWD/inc/gui/apps/manual_control_dialog.h \
    $$PWD/inc/gui/camera/camera_widget.h \
    $$PWD/inc/gui/camera/camera_calib_dialog.h \
    $$PWD/inc/gui/camera/camera_calib_settings_dialog.h \
    $$PWD/inc/gui/camera/camera_calib_app.h \
    $$PWD/inc/gui/misc/scatter3d_widget.h \
    $$PWD/inc/gui/misc/chartview.h \
    $$PWD/inc/gui/misc/file_selection_form.h \
    $$PWD/inc/calib/calib_excitation.h \
    $$PWD/inc/homing/homing_proprioceptive_app.h \
    $$PWD/inc/homing/homing_vision_app.h \
    $$PWD/inc/homing/matlab_thread.h \
    $$PWD/inc/apps/joints_pvt_app.h \
    $$PWD/inc/apps/manual_control_app.h \
    $$PWD/inc/ctrl/controller_base.h \
    $$PWD/inc/ctrl/controller_singledrive.h \
    $$PWD/inc/ctrl/controller_joints_pvt.h \
    $$PWD/inc/ctrl/winch_torque_controller.h \
#    $$PWD/inc/state_estimation/ext_kalman_filter.h \
    $$PWD/inc/utils/types.h \
    $$PWD/inc/utils/macros.h \
    $$PWD/inc/utils/msgs.h \
    $$PWD/inc/utils/easylog_wrapper.h \
    $$PWD/inc/utils/cameraparamsjsonparser.h \
    $$PWD/inc/utils/sensorsconfigjsonparser.h \
    $$PWD/inc/debug/debug_routine.h \
    $$PWD/libs/easyloggingpp/src/easylogging++.h \
    $$PWD/libs/grab_common/grabcommon.h \
    $$PWD/libs/grab_common/pid/pid.h \

SOURCES = \
    $$PWD/src/main.cpp \
#    $$PWD/src/main_cam_test.cpp \
    $$PWD/src/robot/cablerobot.cpp \
    $$PWD/src/robot/components/actuator.cpp \
    $$PWD/src/robot/components/winch.cpp \
    $$PWD/src/robot/components/pulleys_system.cpp \
    $$PWD/src/gui/main_gui.cpp \
    $$PWD/src/gui/login_window.cpp \
    $$PWD/src/gui/calib/calibration_dialog.cpp \
    $$PWD/src/gui/calib/calib_interface_excitation.cpp \
    $$PWD/src/gui/homing/homing_dialog.cpp \
    $$PWD/src/gui/homing/homing_interface.cpp \
    $$PWD/src/gui/homing/homing_interface_vision.cpp \
    $$PWD/src/gui/homing/homing_interface_proprioceptive.cpp \
    $$PWD/src/gui/homing/init_torque_form.cpp \
    $$PWD/src/gui/apps/joints_pvt_dialog.cpp \
    $$PWD/src/gui/apps/manual_control_dialog.cpp \
    $$PWD/src/gui/camera/camera_widget.cpp \
    $$PWD/src/gui/camera/camera_calib_dialog.cpp \
    $$PWD/src/gui/camera/camera_calib_settings_dialog.cpp \
    $$PWD/src/gui/camera/camera_calib_app.cpp \
    $$PWD/src/gui/misc/scatter3d_widget.cpp \
    $$PWD/src/gui/misc/chartview.cpp \
    $$PWD/src/gui/misc/file_selection_form.cpp \
    $$PWD/src/calib/calib_excitation.cpp \
    $$PWD/src/homing/homing_proprioceptive_app.cpp \
    $$PWD/src/homing/homing_vision_app.cpp \
    $$PWD/src/homing/matlab_thread.cpp \
    $$PWD/src/apps/joints_pvt_app.cpp \
    $$PWD/src/apps/manual_control_app.cpp \
    $$PWD/src/ctrl/controller_base.cpp \
    $$PWD/src/ctrl/controller_singledrive.cpp \
    $$PWD/src/ctrl/controller_joints_pvt.cpp \
    $$PWD/src/ctrl/winch_torque_controller.cpp \
#    $$PWD/src/state_estimation/ext_kalman_filter.cpp \
    $$PWD/src/utils/msgs.cpp \
    $$PWD/src/utils/easylog_wrapper.cpp \
    $$PWD/src/utils/cameraparamsjsonparser.cpp \
    $$PWD/src/utils/sensorsconfigjsonparser.cpp \
    $$PWD/src/debug/debug_routine.cpp \
    $$PWD/libs/easyloggingpp/src/easylogging++.cc \
    $$PWD/libs/grab_common/grabcommon.cpp \
    $$PWD/libs/grab_common/pid/pid.cpp \

FORMS += \
    $$PWD/widgets/main_gui.ui \
    $$PWD/widgets/login_window.ui \
    $$PWD/widgets/calib/calibration_dialog.ui \
    $$PWD/widgets/calib/calib_interface_excitation.ui \
    $$PWD/widgets/homing/homing_dialog.ui \
    $$PWD/widgets/homing/homing_interface_proprioceptive.ui\
    $$PWD/widgets/homing/homing_interface_vision.ui \
    $$PWD/widgets/homing/init_torque_form.ui \
    $$PWD/widgets/camera/camera_widget.ui \
    $$PWD/widgets/camera/camera_calib_dialog.ui \
    $$PWD/widgets/camera/camera_calib_app.ui \
    $$PWD/widgets/camera/camera_calib_settings_dialog.ui \
    $$PWD/widgets/apps/joints_pvt_dialog.ui \
    $$PWD/widgets/apps/manual_control_dialog.ui \
    $$PWD/widgets/misc/scatter3d_widget.ui \
    $$PWD/widgets/misc/file_selection_form.ui \

RESOURCES += \
    resources/resources.qrc

INCLUDEPATH += \
    $$PWD/inc \
    $$PWD/libs/grab_common \
    $$PWD/libs/easyloggingpp/src

QT += core gui widgets multimedia multimediawidgets datavisualization charts

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
DEFINES += USE_QT=1
DEFINES += DEBUG_GUI=0

# GRAB Ethercat lib
unix:!macx: LIBS += -L$$PWD/libs/grab_common/libgrabec/lib/ -lgrabec
INCLUDEPATH += $$PWD/libs/grab_common/libgrabec \
    libs/grab_common/libgrabec/inc
DEPENDPATH += $$PWD/libs/grab_common/libgrabec
unix:!macx: PRE_TARGETDEPS += $$PWD/libs/grab_common/libgrabec/lib/libgrabec.a

# GRAB Real-time lib
unix:!macx: LIBS += -L$$PWD/libs/grab_common/libgrabrt/lib/ -lgrabrt
INCLUDEPATH += $$PWD/libs/grab_common/libgrabrt \
    libs/grab_common/libgrabrt/inc
DEPENDPATH += $$PWD/libs/grab_common/libgrabrt
unix:!macx: PRE_TARGETDEPS += $$PWD/libs/grab_common/libgrabrt/lib/libgrabrt.a

# EtherCAT lib
INCLUDEPATH += /opt/etherlab/include
DEPENDPATH  += /opt/etherlab/lib/
LIBS        += /opt/etherlab/lib/libethercat.a

# State machine lib
unix:!macx: LIBS += -L$$PWD/libs/state_machine/lib/ -lstate_machine
INCLUDEPATH += $$PWD/libs/state_machine $$PWD/libs/state_machine/inc
DEPENDPATH += $$PWD/libs/state_machine
unix:!macx: PRE_TARGETDEPS += $$PWD/libs/state_machine/lib/libstate_machine.a

# GRAB CDPR lib
unix:!macx: LIBS += -L$$PWD/libs/grab_common/libcdpr/lib/ -lcdpr
INCLUDEPATH += $$PWD/libs/grab_common/libcdpr \
    $$PWD/libs/grab_common/libcdpr/inc \
    $$PWD/libs/grab_common/libcdpr/tools
DEPENDPATH += $$PWD/libs/grab_common/libcdpr
unix:!macx: PRE_TARGETDEPS += $$PWD/libs/grab_common/libcdpr/lib/libcdpr.a

# Geometric lib
unix:!macx: LIBS += -L$$PWD/libs/grab_common/libgeom/lib/ -lgeom
INCLUDEPATH += $$PWD/libs/grab_common/libgeom $$PWD/libs/grab_common/libgeom/inc/
DEPENDPATH += $$PWD/libs/grab_common/libgeom
unix:!macx: PRE_TARGETDEPS += $$PWD/libs/grab_common/libgeom/lib/libgeom.a

# Numeric lib
unix:!macx: LIBS += -L$$PWD/libs/grab_common/libnumeric/lib/ -lnumeric
INCLUDEPATH += $$PWD/libs/grab_common/libnumeric \
    $$PWD/libs/grab_common/libnumeric/inc/
DEPENDPATH += $$PWD/libs/grab_common/libnumeric
unix:!macx: PRE_TARGETDEPS += $$PWD/libs/grab_common/libnumeric/lib/libnumeric.a

# OpenCV lib
INCLUDEPATH += /usr/local/lib/opencv-4.0.1/build/include \
               /usr/local/include/opencv4
LIBS += -L"/usr/local/lib/opencv-4.0.1/build/lib"
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
