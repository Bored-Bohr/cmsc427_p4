TEMPLATE = app
CONFIG += qt warn_on release embed_manifest_exe
CONFIG -= app_bundle
QT += gui opengl xml widgets
FORMS += cmsc427.ui
SOURCES += GLview.cpp cmsc427.cpp Mesh.cpp
HEADERS += GLview.hpp cmsc427.hpp Mesh.hpp
QMAKE_CXXFLAGS += -I/usr/local/include
unix {
QMAKE_CXXFLAGS_WARN_ON += -Wno-unknown-pragmas
}
RESOURCES += resources.qrc
OTHER_FILES += perfrag.fsh perfrag.vsh texture.fsh texture.vsh
