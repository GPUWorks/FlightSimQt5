#-------------------------------------------------
#
# Project created by QtCreator 2013-04-05T19:35:15
#
#-------------------------------------------------
DEFINES += QWT_DLL  # Necesario para usar clases derivadas OBJECT, como "attitude_indicator"

QT       += core gui svg serialport widgets


TARGET = FlightSim
TEMPLATE = app


SOURCES += main.cpp\
        guipanel.cpp \
    crc.c \
    protocol.c

HEADERS  += guipanel.h \
    crc.h \
    protocol.h

FORMS    += guipanel.ui
CONFIG   +=qwt
#CONFIG   +=console# Añadir para usar la consola de depuracion --> Window-Views-Debugger Log


unix{
    LIBS +=  -lanalogwidgets -lqwt

}

win32 {

LIBS += -lanalogwidgetsd -lqwtd

}

QMAKE_LFLAGS_RELEASE ="/OPT:REF" # Evita el molesto problema de Debug/xxx.exe

RESOURCES += \
    images.qrc

OTHER_FILES +=
