QT       += core gui
QT       += concurrent
QT += multimedia

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QT += charts

CONFIG += c++20

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += $$PWD/include
LIBS += -L$$PWD -lfftw3f-3 -lfftw3-3 -lfftw3l-3

SOURCES += \
    CarAxisSelect.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    CarAxis.h \
    mainwindow.h \
    sensor_data_processor.h

FORMS += \
    mainwindow.ui

QMAKE_CXXFLAGS += -fopenmp
#QMAKE_CXXFLAGS += -ftree-vectorize -mavx2 -msse4.1
LIBS += -lgomp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
