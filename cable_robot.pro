HEADERS = \
   $$PWD/header/interface/actuatorinterface.h \
   $$PWD/header/interface/actuatorpvtinterface33.h \
   $$PWD/header/interface/cablerobotinterface.h \
   $$PWD/header/interface/calibrationinterface.h \
   $$PWD/header/interface/demointerface33.h \
   $$PWD/header/interface/demointerface66.h \
   $$PWD/header/interface/hominginterface.h \
   $$PWD/header/interface/manualinterface33.h \
   $$PWD/header/interface/manualinterface66.h \
   $$PWD/header/cablerobot.h \
   $$PWD/header/cablerobotmaster.h \
   $$PWD/header/easycatslave.h \
   $$PWD/header/ethercatmaster.h \
   $$PWD/header/ethercatslave.h \
   $$PWD/header/goldsolowhistledrive.h \
   $$PWD/header/servomotor.h \
   $$PWD/header/winch.h

SOURCES = \
   $$PWD/src/interface/actuatorinterface.cpp \
   $$PWD/src/interface/actuatorpvtinterface33.cpp \
   $$PWD/src/interface/cablerobotinterface.cpp \
   $$PWD/src/interface/calibrationinterface.cpp \
   $$PWD/src/interface/demointerface33.cpp \
   $$PWD/src/interface/demointerface66.cpp \
   $$PWD/src/interface/hominginterface.cpp \
   $$PWD/src/interface/manualinterface33.cpp \
   $$PWD/src/interface/manualinterface66.cpp \
   $$PWD/src/cablerobot.cpp \
   $$PWD/src/cablerobotmaster.cpp \
   $$PWD/src/easycatslave.cpp \
   $$PWD/src/ethercatmaster.cpp \
   $$PWD/src/goldsolowhistledrive.cpp \
   $$PWD/src/main.cpp \
   $$PWD/src/servomotor.cpp \
    src/ethercatslave.cpp \
    src/winch.cpp

INCLUDEPATH = \
    $$PWD/header \
    $$PWD/header/interface

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
