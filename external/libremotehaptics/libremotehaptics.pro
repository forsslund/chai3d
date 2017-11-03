#-------------------------------------------------
#
# Project created by QtCreator 2012-06-14T16:24:57
#
#-------------------------------------------------

QT       += network

QT       -= gui

TARGET = remotehaptics
CONFIG += staticlib
TEMPLATE = lib

SOURCES += libremotehaptics.cpp \
    networkhandler.cpp

HEADERS += libremotehaptics.h \
    networkhandler.h
unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
