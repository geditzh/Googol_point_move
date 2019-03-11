#-------------------------------------------------
#
# Project created by QtCreator 2018-08-08T18:32:25
#
#-------------------------------------------------

#QT       += core gui
QT += core
QT -= gui

#greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = googol
TEMPLATE = app
CONFIG += c++11

CONFIG += console
CONFIG -= app_bundle

SOURCES += main.cpp\
    AssemblyRobot.cpp

HEADERS  += \
    AssemblyRobot.h \
    RobotDefine.h

FORMS    += mainwindow.ui

LIBS += -LC:\Users\df\Desktop\Googol_point_move\googol -lgts
LIBS += -LC:\Users\df\Desktop\Googol_point_move\googol -lExtMdl
