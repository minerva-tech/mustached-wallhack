TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

LIBS += -lopencv_core \
        -lopencv_highgui \
        -lopencv_features2d \
        -lopencv_flann \
        -lopencv_nonfree \
        -lopencv_calib3d \
        -lopencv_imgproc \
        -lopencv_video \
        -lGL -lglut -lGLU

SOURCES += main.cpp \
    display3d.cpp

HEADERS += \
    display3d.h

QMAKE_CXXFLAGS += -std=c++11
