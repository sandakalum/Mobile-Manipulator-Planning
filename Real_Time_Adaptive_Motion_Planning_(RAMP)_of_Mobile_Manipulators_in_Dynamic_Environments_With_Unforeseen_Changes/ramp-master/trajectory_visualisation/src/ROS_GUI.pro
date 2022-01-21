#-------------------------------------------------
#
# Project created by QtCreator 2011-02-17T16:31:49
#
#-------------------------------------------------

QT += core gui
QT += webkit
QT += opengl

win32:LIBS += C:/Qt/2010.05/mingw/lib/libwinmm.a

RESOURCES += ../resources/images.qrc

TARGET = ROS_GUI
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    joystick.cpp \
    Ros.cpp \
    ArmCircleWidget.cpp \
    urg_ticks.cpp \
    ticks.cpp \
    ImageBig.cpp \
    TrajectoryView.cpp

HEADERS  += mainwindow.h \
    ArmWidget.h \
    joint.h \
    GripperWidget.h \
    WristWidget.h \
    Line.h \
    GPS.h \
    joystick.h \
    Ros.h \
    ArmCircleWidget.h \
    Point.h \
    urg_ticks.h \
    ticks.h \
    Color.h \
    Image.h \
    ImageBig.h \
    Hokuyo.h \
    Hokuyo_Points.h \
    MyQGraphicsView.h \
    ../include/Ros.h \
    ../include/mainwindow.h \
    ../include/TrajectoryView.h

FORMS    += ../ui/mainwindow.ui
