TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv4

# FUZZYLITE PATH THOMAS
INCLUDEPATH += /home/th/Documents/Cpp/lib/fuzzylite-6.0/fuzzylite
LIBS += /home/th/Documents/Cpp/lib/fuzzylite-6.0/fuzzylite/release/bin/libfuzzylite-static.a
LIBS += /home/th/Documents/Cpp/lib/fuzzylite-6.0/fuzzylite/release/bin/libfuzzylite.so.6.0
LIBS += /home/th/Documents/Cpp/lib/fuzzylite-6.0/fuzzylite/release/bin/libfuzzylite.so
