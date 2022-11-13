QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


LIBS += /usr/local/lib/libosqp.so \
        /usr/local/lib/libOsqpEigen.so

SOURCES += main.cpp \
        tension_smoother_2.cpp \
        reference_path.cpp \
        reference_path_impl.cpp \
        tools/tools.cpp \
        tools/spline.cpp \
        tinyspline_ros/src/tinyspline_ros/tinysplinecpp.cpp \
        tinyspline_ros/src/tinyspline_ros/tinyspline.c

HEADERS += \
        tension_smoother_2.hpp \
        tinyspline_ros/include/tinyspline_ros/tinysplinecpp.h \
        tinyspline_ros/include/tinyspline_ros/tinyspline.h \
        reference_path.hpp \
        reference_path_impl.hpp \
        tools/tools.hpp \
        tools/spline.h \
        tinyspline_ros/include/tinyspline_ros/tinysplinecpp.h \
        tinyspline_ros/include/tinyspline_ros/tinyspline.h \
        tools/spline.h

INCLUDEPATH += osqp-eigen/include/tinyspline_ros/ \
               osqp-eigen/include/ \
               eigen/ \
               tinyspline_ros/ \
               osqp/include/ \
