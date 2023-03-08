#-------------------------------------------------
#
# Project created by QtCreator 2017-11-26T15:11:30
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG += c++11
TARGET = test
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib \
 -lopencv_core \
 -lopencv_highgui \
 -lopencv_imgproc \
 -lopencv_features2d \
 -lopencv_calib3d \
 -lopencv_legacy \
 -lopencv_nonfree \
 -lopencv_contrib

INCLUDEPATH += /usr/include/boost/
LIBS += -L/usr/lib \
 -lboost_system \
 -lboost_filesystem \
 -lboost_thread

INCLUDEPATH += /usr/include/eigen3/ \
 /usr/include/pcl-1.7/
LIBS += -lpcl_common \
 -lpcl_io \
 -lpcl_search \
 -lpcl_features \
 -lpcl_keypoints \
 -lpcl_filters \
 -lpcl_kdtree \
 -lpcl_visualization
#LIBS += -lpcl_registration \
# -lpcl_sample_consensus \
# -lpcl_features \
# -lpcl_filters \
# -lpcl_surface \
# -lpcl_segmentation \
# -lpcl_search \
# -lpcl_kdtree \
# -lpcl_octree \
# -lpcl_common \
# -lpcl_io \
# -lpcl_visualization

INCLUDEPATH += /usr/include/vtk-6.2/
LIBS += ${VTK_LIBRARIES}
#LIBS += -L/usr/lib/x86_64-linux-gnu \
#    ${VTK_LIBRARIES}

# -lvtkFiltering \
# -lvtkIO
#INCLUDEPATH += /usr/include/vtk-6.2/
#LIBS += -L/usr/lib \
# -lvtkCommon \
# -lvtksys \
# -lvtkViews \
# -lvtkWidgets \
# -lvtkRendering \
# -lvtkGraphics \
# -lvtkImaging \
# -lvtkIO \
# -lvtkFiltering \
# -lvtkDICOMParser \
# -lvtkmetaio \
# -lvtkexoIIc \
# -lvtkftgl \
# -lvtkHybrid

#INCLUDEPATH += /usr/include/flann/
#LIBS += -lflann_cpp

SOURCES += main.cpp \
    brand.cpp \
    input.cpp \
    match.cpp \
    detector.cpp \
    extractor.cpp \
    pipeline.cpp \
    database.cpp

HEADERS  += \
    brand.h \
    input.h \
    match.h \
    detector.h \
    extractor.h \
    pipeline.h \
    database.h

FORMS    +=
