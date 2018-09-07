
HEADERS += inc/matrix.h inc/solvers.h \
    inc/common.h

SOURCES += src/matrix.cpp src/solvers.cpp

INCLUDEPATH += inc src

QT       -= gui

CONFIG   += c++11 staticlib
CONFIG   -= app_bundle

TEMPLATE = lib

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
