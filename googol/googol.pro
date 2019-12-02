#-------------------------------------------------
#
# Project created by QtCreator 2018-08-08T18:32:25
#
#-------------------------------------------------

#QT       += core gui
QT += core
QT -= gui

#greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
#工程配置相关
TARGET = googol
TEMPLATE = app
CONFIG += c++11

CONFIG += console
CONFIG -= app_bundle

#添加头文件与源文件
SOURCES += main.cpp\
    AssemblyRobot.cpp

HEADERS  += \
    AssemblyRobot.h \
    RobotDefine.h

#添加固高静态库 gts.lib ExtMdl.lib。（工程栏右键添加外部库）
win32: LIBS += -L$$PWD/./ -lgts -lExtMdl

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.
