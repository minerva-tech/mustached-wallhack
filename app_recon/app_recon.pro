TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

LIBS += -lopencv_core -lopencv_highgui -lopencv_features2d -lGL -lglut -lGLU

SOURCES += main.cpp \
    display3d.cpp

HEADERS += \
    display3d.h

QMAKE_CXXFLAGS += -std=c++11
