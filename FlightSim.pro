#-------------------------------------------------
#
# Project created by QtCreator 2013-04-05T19:35:15
#
#-------------------------------------------------
DEFINES += QWT_DLL  # Necesario para usar clases derivadas OBJECT, como "attitude_indicator"

QT       += core gui svg serialport widgets


TARGET = FlightSim
TEMPLATE = app

win32: DEFINES += WIN32 _WINDOWS _USE_MATH_DEFINES

win32:CONFIG(release, debug|release):    DEFINES += NDEBUG
else:win32:CONFIG(debug, debug|release): DEFINES += _DEBUG



SOURCES += main.cpp\
        guipanel.cpp \
        crc.c \
        protocol.c \
        qfi_PFD.cpp \
        WidgetPFD.cpp \
        LayoutSquare.cpp

HEADERS  += guipanel.h \
            crc.h \
            protocol.h \
            qfi_PFD.h \
            WidgetPFD.h \
            LayoutSquare.h

FORMS    += guipanel.ui \
    WidgetPFD.ui
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
    images.qrc \
    qfi.qrc


