#ifndef __EDGEGUI_HPP__
#define __EDGEGUI_HPP__

#include <QtGui>
#include <QtOpenGL>

#include "ui_cmsc427.h"

class CMSC427Win : public QMainWindow, private Ui::CMSC427Win {
    Q_OBJECT
public:
    CMSC427Win(QMainWindow *parent = 0);
    ~CMSC427Win() {  }
protected:
    void keyPressEvent( QKeyEvent* e ) {  glwidget->keyPressGL(e);  }
    void open_OBJ_file();
    private slots:
    // Menu triggers
    void on_action_Quit_triggered() { QCoreApplication::instance()->quit(); } // Quit.
    void on_action_Open_triggered();
    void on_pushOpenOBJFile_clicked(bool);
    void on_pushLightMotion_clicked(bool) { glwidget->toggleLightMotion(); }
};



#endif
