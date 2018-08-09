HEADERS = \
    header/comm/master/cablerobotmaster.h \
    header/comm/master/ethercatmaster.h \
    header/comm/slave/easycatslave.h \
    header/comm/slave/ethercatslave.h \
    header/comm/slave/goldsolowhistledrive.h \
    header/components/cablerobot.h \
    header/components/servomotor.h \
    header/components/winch.h \
    header/interface/actuatorinterface.h \
    header/interface/actuatorpvtinterface33.h \
    header/interface/cablerobotinterface.h \
    header/interface/calibrationinterface.h \
    header/interface/demointerface33.h \
    header/interface/demointerface66.h \
    header/interface/hominginterface.h \
    header/interface/manualinterface33.h \
    header/interface/manualinterface66.h \
    header/common.h \

SOURCES = \
    src/comm/master/cablerobotmaster.cpp \
    src/comm/master/ethercatmaster.cpp \
    src/comm/slave/easycatslave.cpp \
    src/comm/slave/ethercatslave.cpp \
    src/comm/slave/goldsolowhistledrive.cpp \
    src/components/cablerobot.cpp \
    src/components/servomotor.cpp \
    src/components/winch.cpp \
    src/interface/actuatorinterface.cpp \
    src/interface/actuatorpvtinterface33.cpp \
    src/interface/cablerobotinterface.cpp \
    src/interface/calibrationinterface.cpp \
    src/interface/demointerface33.cpp \
    src/interface/demointerface66.cpp \
    src/interface/hominginterface.cpp \
    src/interface/manualinterface33.cpp \
    src/interface/manualinterface66.cpp \
    src/main.cpp \

INCLUDEPATH = \
    $$PWD/header \
    $$PWD/header/interface \
    $$PWD/header/comm/master \
    $$PWD/header/comm/slave \
    $$PWD/header/components \

FORMS += \
    gui/sub_win/actuatorinterface.ui \
    gui/sub_win/actuatorpvtinterface33.ui \
    gui/sub_win/calibrationinterface.ui \
    gui/sub_win/demointerface33.ui \
    gui/sub_win/demointerface66.ui \
    gui/sub_win/hominginterface.ui \
    gui/sub_win/manualinterface33.ui \
    gui/sub_win/manualinterface66.ui \
    gui/cablerobotinterface.ui

QT += core gui widgets

CONFIG += c++11 console
CONFIG -= app_bundle

INCLUDEPATH += /opt/etherlab/include/
DEPENDPATH  += /opt/etherlab/lib/
LIBS        += /opt/etherlab/lib/libethercat.a

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
