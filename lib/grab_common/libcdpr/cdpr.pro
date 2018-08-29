
HEADERS += inc/kinematics.h

SOURCES += src/kinematics.cpp

INCLUDEPATH += inc \
        ../libgeom/inc \
        ../libnumeric/inc \
        ../libnumeric/src

LIBS += ../libgeom/build/libgeom.a ../libnumeric/build/libnumeric.a

QT       -= gui

CONFIG   += staticlib c++11
CONFIG   -= app_bundle

TEMPLATE = lib

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
