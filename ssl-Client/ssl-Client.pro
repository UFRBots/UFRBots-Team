TARGET = main
TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
#CONFIG -= qt

QT += core
QT += network

SOURCES += main.cpp \
    include/vssref_command.pb.cc \
    include/vssref_common.pb.cc \
    include/vssref_placement.pb.cc \
    net/pb/command.pb.cc \
    net/pb/common.pb.cc \
    net/pb/packet.pb.cc \
    net/pb/replacement.pb.cc \
    net/robocup_ssl_client.cpp \
    net/netraw.cpp \
    net/grSim_client.cpp

HEADERS += net/pb/command.pb.h \
    include/timer.h \
    include/vssref_command.pb.h \
    include/vssref_common.pb.h \
    include/vssref_placement.pb.h \
    net/pb/common.pb.h \
    net/pb/packet.pb.h \
    net/pb/replacement.pb.h \
    net/robocup_ssl_client.h \
    net/netraw.h \
    net/grSim_client.h

LIBS += -lpthread \
    -L/usr/local/lib -lprotobuf -lz

DISTFILES += \
    include/protobuf/vssreferee/protobuf.sh \
    include/protobuf/vssreferee/vssref_command.proto \
    include/protobuf/vssreferee/vssref_common.proto \
    include/protobuf/vssreferee/vssref_placement.proto
