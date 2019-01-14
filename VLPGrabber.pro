#-------------------------------------------------
#
# Project created by QtCreator 2016-11-19T21:00:34
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = VLPGrabber
CONFIG   += console
CONFIG   -= app_bundle

QMAKE_CXXFLAGS+=-std=c++11#enable c++11 feature
#LIBS+=libws2_32#if not on windows+mingw platform，disable it

TEMPLATE = app
SOURCES += main.cpp VLPGrabber.cpp

HEADERS += \
    VLPGrabber.h
