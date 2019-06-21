# -------------------------------------------------
# Project created by QtCreator 2014-03-18T10:40:12
# -------------------------------------------------
TARGET = QT_Demo
TEMPLATE = app

INCLUDEPATH += "../../include/"
LIBS += -lMVSDK -lprotobuf

SOURCES += main.cpp \
    mainwindow.cpp \
    capturethread.cpp \
    ./proto/MVCamera_para.pb.cc \
    protoloader.cpp
HEADERS += mainwindow.h \
    capturethread.h \
    ./proto/MVCamera_para.pb.h \
    protoloader.h
FORMS += mainwindow.ui
