HEADERS = \
    header/comm/cablerobotmaster.h \
    header/comm/easycatslave.h \
    header/comm/ethercatmaster.h \
    header/comm/ethercatslave.h \
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
    header/cablerobot.h \
    header/goldsolowhistledrive.h \
    header/common.h

SOURCES = \
    src/comm/cablerobotmaster.cpp \
    src/comm/easycatslave.cpp \
    src/comm/ethercatmaster.cpp \
    src/comm/ethercatslave.cpp \
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
    src/cablerobot.cpp \
    src/goldsolowhistledrive.cpp \
    src/main.cpp

INCLUDEPATH = \
    $$PWD/header \
    $$PWD/header/interface \
    $$PWD/header/comm \
    $$PWD/header/components

FORMS += \
   $$PWD/gui/actuatorinterface.ui \
   $$PWD/gui/actuatorpvtinterface33.ui \
   $$PWD/gui/cablerobotinterface.ui \
   $$PWD/gui/calibrationinterface.ui \
   $$PWD/gui/demointerface33.ui \
   $$PWD/gui/demointerface66.ui \
   $$PWD/gui/hominginterface.ui \
   $$PWD/gui/manualinterface33.ui \
   $$PWD/gui/manualinterface66.ui \

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
