#ifndef __GLVIEW_HPP__
#define __GLVIEW_HPP__

#include <QtGui>
#include <QOpenGLWidget>

#include "Mesh.hpp"

class GLview : public QOpenGLWidget, protected QOpenGLFunctions  {
    Q_OBJECT
public:
    GLview(QWidget *parent = 0);
    ~GLview();
    void keyPressGL(QKeyEvent* e);
    bool LoadOBJFile(const QString file, const QString path);
    void toggleLightMotion();
protected:
    void initializeGL(); // QGLWidget OpenGL interface
    void initLightCameraGL(Mesh* mesh, long i);
    void paintGL();
    void resizeGL(int width, int height);


//    float t = 0.01;
//    bool tag = false, tag2 = false;
    QVector3D e, lc,lu;
    vector<QVector3D> positions;
    vector<QQuaternion> orientations;

    void randCoordOrient(QVector3D &position, QQuaternion &orientation);

    //Mesh *mesh;  // Geometry data.
    vector<Mesh*> meshList;
    QOpenGLShaderProgram phong_shader, texture_shader;

    // Camera parameters for the view frustum --------------
    QVector3D eye, lookCenter, lookUp;
    QQuaternion camrot, camrot2;
    float yfov, neardist, fardist;

    // Light animation speed.
    float latitude_velocity;

    // Light parameters ------
    QVector3D LightPosition, LightIntensity;

    QOpenGLVertexArrayObject vao;
    bool prepareShaderProgram(QOpenGLShaderProgram &shader, const QString &vertex_file, const QString &fragment_file);
    bool prepareShaderProgramTex();

    // Shadow parameters
    GLuint shadow_FBO, depthTex;
    int shadowmap_width, shadowmap_height; // power of 2 is best
    void initializeShadowMapGL();

    // Used to track mouse position for keyboard shortcuts.
    bool lastPosFlag; // Flag is true if a last position has been captured. Set to false
    // prior to mouse driven interaction.
    float lastPosX, lastPosY;   // Mouse state information.
    bool scaleFlag, translateFlag, rotateFlag;  // Camera movement state information.

    bool lightMotionFlag, modelSlerpFlag;

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void toggleRotate();
    void toggleScale();
    void toggleTranslate();

    QElapsedTimer elapsed_time;
    double timeAccumulator, totalTime;
    void updateGLview(float dt);
    void timerEvent(QTimerEvent *event);
};

#endif
