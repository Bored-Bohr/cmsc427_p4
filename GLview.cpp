#include "GLview.hpp"

#include <iostream>
#include <algorithm>
#include <cmath>

using namespace std;

float copysign0(float x, float y) { return (y == 0.0f) ? 0 : copysign(x,y); }

void Matrix2Quaternion(QQuaternion &Q, QMatrix4x4 &M) {
    Q.setScalar(sqrt( max( 0.0f, 1 + M(0,0)  + M(1,1) + M(2,2) ) ) / 2);
    Q.setX(sqrt( max( 0.0f, 1 + M(0,0) - M(1,1) - M(2,2) ) ) / 2);
    Q.setY(sqrt( max( 0.0f, 1 - M(0,0) + M(1,1) - M(2,2) ) ) / 2);
    Q.setZ(sqrt( max( 0.0f, 1 - M(0,0) - M(1,1) + M(2,2) ) ) / 2);
    Q.setX(copysign0( Q.x(), M(2,1) - M(1,2) ));  Q.setY(copysign0( Q.y(), M(0,2) - M(2,0) ));  Q.setZ(copysign0( Q.z(), M(1,0) - M(0,1) )) ;
}

// GLView constructor. DO NOT MODIFY.
GLview::GLview(QWidget *parent)  : QOpenGLWidget(parent) {
    // start off with all transformation flags disabled
    scaleFlag = translateFlag = rotateFlag = false; lastPosFlag = false;
    mesh = NULL;
    startTimer(20,Qt::PreciseTimer);

    elapsed_time.start();
    elapsed_time.invalidate();
    timeAccumulator = 0;
    totalTime = 0;

    lightMotionFlag = false;
    latitude_velocity = 20;
}

GLview::~GLview() {
    makeCurrent();
    if(mesh != NULL) delete mesh;
    doneCurrent();
}

// Load object file.
bool GLview::LoadOBJFile(const QString file, const QString path) {
    makeCurrent();
    Mesh *newmesh = new Mesh;
    if(!newmesh->load_obj(file, path)) {
        delete newmesh;
        return false;
    }
    if(mesh != NULL) {
        delete mesh;
    }
    mesh = newmesh;
    mesh->storeVBO();
    doneCurrent();
    return true;
}

// Set default GL parameters.
void GLview::initializeGL() {
    initializeOpenGLFunctions();
    vao.create();
    if (vao.isCreated()) {
        vao.bind();
    }
    glClearColor( 0.15, 0.15, 0.15, 1.0f );   // Set the clear color to black
    glEnable(GL_DEPTH_TEST);    // Enable depth buffer

    // Prepare a complete shader program...exit on failure
    if ( !prepareShaderProgram(phong_shader,  ":/perfrag.vsh", ":/perfrag.fsh" ) ) return;

    // Set default lighting parameters.
    LightPosition = QVector3D(-2,-2, 3);
    LightIntensity = QVector3D(1,1,1);

    // Initialize default camera parameters
    yfov = 55;
    neardist = 1; fardist = 1000;
    eye = QVector3D(-3,3,3); lookCenter = QVector3D(0,0,0); lookUp = QVector3D(0,0,1);

    QMatrix4x4 view;
    view.lookAt(eye, lookCenter, lookUp);
    Matrix2Quaternion(camrot, view);
    initializeShadowMapGL();
}

// I've provided you with a function for creating a frame buffer backed by a depth texture.
void GLview::initializeShadowMapGL() {
    // The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
    glGenFramebuffers(1, &shadow_FBO);
    glBindFramebuffer(GL_FRAMEBUFFER, shadow_FBO);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    // Depth texture Slower than a depth buffer, but you can sample it later in your shader
    shadowmap_width = 1024;
    shadowmap_height = 1024;

    glGenTextures(1, &depthTex);
    glBindTexture(GL_TEXTURE_2D, depthTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, shadowmap_width, shadowmap_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
    GLfloat border[] = {1.0f, 0.0f,0.0f,0.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTex, 0);
//    glDrawBuffer(GL_NONE);
//    glReadBuffer(GL_NONE);

    GLenum result = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if( result == GL_FRAMEBUFFER_COMPLETE) {
        cout << "Framebuffer is complete." << endl;
    }
    else if(result == GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT) {
        cout << "incomplete attach." << endl; exit(1);
    }
    else {
        cout << "Framebuffer is not complete" << endl; exit(1);
    }

    // Bind main framebuffer again.
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
}


// Set the viewport to window dimensions. DO NOT MODIFY.
void GLview::resizeGL( int w, int h ) {
    glViewport( 0, 0, w, qMax( h, 1 ) );
}


// Set the model view matrix (MVP) for the light and camera
void GLview::initLightCameraGL() {
    QMatrix4x4 model, view, projection;
    model.scale(1.0);
    view.rotate(camrot);
    view.translate(-eye);
    projection.perspective(yfov, (float)width() / (float)height(), neardist, fardist);

    QMatrix4x4 model_view = view * model;
    QMatrix3x3 normal_matrix = model_view.normalMatrix();
    QMatrix4x4 MVP = projection * model_view;

    phong_shader.bind();
    phong_shader.setUniformValue("ModelViewMatrix", model_view);
    phong_shader.setUniformValue("NormalMatrix", normal_matrix);
    phong_shader.setUniformValue("MVP", MVP);
    phong_shader.setUniformValue("LightIntensity", LightIntensity);
    phong_shader.setUniformValue("LightPosition", view * QVector4D(LightPosition, 1));
}

void GLview::paintGL() {
    if(mesh == NULL) return;
    initLightCameraGL(); // Update lighting and camara position.

    // Bind shadow map buffer.
    glBindFramebuffer(GL_FRAMEBUFFER, shadow_FBO);
    glClear( GL_DEPTH_BUFFER_BIT );
    glViewport(0, 0, shadowmap_width, shadowmap_height );

    // Hint: something needs to happen here.



    // Bind defualt window frame buffer.
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
    glViewport( 0.0, 0.0, (double)devicePixelRatio()*width(), qMax( (double)devicePixelRatio()*height(), 1.0 ) );

    // Clear the buffer with the current clearing color
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Update VBOs associated with phong shader.
    phong_shader.bind();

    mesh->vertexBuffer.bind();
    phong_shader.setAttributeBuffer( "VertexPosition", GL_FLOAT, 0, 3 );
    phong_shader.enableAttributeArray( "VertexPosition" );

    mesh->normalBuffer.bind();
    phong_shader.setAttributeBuffer( "VertexNormal", GL_FLOAT, 0, 3 );
    phong_shader.enableAttributeArray( "VertexNormal" );

    mesh->kdBuffer.bind();
    phong_shader.setAttributeBuffer( "KdIn", GL_FLOAT, 0, 3 );
    phong_shader.enableAttributeArray( "KdIn" );

    mesh->kaBuffer.bind();
    phong_shader.setAttributeBuffer( "KaIn", GL_FLOAT, 0, 3 );
    phong_shader.enableAttributeArray( "KaIn" );

    mesh->ksBuffer.bind();
    phong_shader.setAttributeBuffer( "KsIn", GL_FLOAT, 0, 3 );
    phong_shader.enableAttributeArray( "KsIn" );

    mesh->shininessBuffer.bind();
    phong_shader.setAttributeBuffer( "ShininessIn", GL_FLOAT, 0, 1 );
    phong_shader.enableAttributeArray( "ShininessIn" );
//    phong_shader.setUniformValue("Kd", mesh->materials[0].Kd);
//    phong_shader.setUniformValue("Ks", mesh->materials[0].Ks);
//    phong_shader.setUniformValue("Ka", mesh->materials[0].Ka);
//    phong_shader.setUniformValue("Shininess", mesh->materials[0].Ns);

    // Hint: texture unit 0 is free for your textures. I've bound the shadow map to texture unit 1.
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, depthTex);

    glDrawArrays( GL_TRIANGLES, 0, 3*mesh->faces.size() );
}

void GLview::keyPressGL(QKeyEvent* e) {
    switch ( e->key() ) {
        case Qt::Key_Escape:
        QCoreApplication::instance()->quit();
        break;
        // Set rotate, scale, translate modes (for track pad mainly).
        case Qt::Key_R:
            toggleRotate();
            break;
        case Qt::Key_S:
            toggleScale();
            break;
        case Qt::Key_T:
            toggleTranslate();
            break;
        default:
            QOpenGLWidget::keyPressEvent( e );
            break;
    }
}


// Compile shaders. DO NOT MODIFY.
bool GLview::prepareShaderProgram(QOpenGLShaderProgram &prep_shader, const QString &vertex_file, const QString &fragment_file) {
    // First we load and compile the vertex shader.
    bool result = prep_shader.addShaderFromSourceFile( QOpenGLShader::Vertex, vertex_file );
    if ( !result ) {
        qWarning() << prep_shader.log();
    }

    // now load and compile the fragment shader...
    result = prep_shader.addShaderFromSourceFile( QOpenGLShader::Fragment, fragment_file );
    if ( !result ) qWarning() << prep_shader.log();

    // now link them to resolve any references.
    result = prep_shader.link();
    if ( !result ) {
        qWarning() << "Could not link shader program:" << prep_shader.log();
        exit(1);
    }
    return result;
}

// Store mouse press position. DO NOT MODIFY
void GLview::mousePressEvent(QMouseEvent *event) {
    if(mesh == NULL) return;
    float px = event->x(), py = event->y();
    float x = 2.0 * (px + 0.5) / float(width())  - 1.0,  y = -(2.0 * (py + 0.5) / float(height()) - 1.0);
    lastPosX = x; lastPosY = y;
    lastPosFlag = true;
    event->accept();
}

// Update camera position on mouse movement. DO NOT MODIFY.
void GLview::mouseMoveEvent(QMouseEvent *event) {
    if(mesh == NULL) return;

    float px = event->x(), py = event->y();
    float x = 2.0 * (px + 0.5) / float(width())  - 1.0;
    float y = -(2.0 * (py + 0.5) / float(height()) - 1.0);

    // Record a last position if none has been set.
    if(!lastPosFlag) {
        lastPosX = x;
        lastPosY = y;
        lastPosFlag = true;
        return;
    }
    float dx = x - lastPosX, dy = y - lastPosY;
    // remember mouse position
    lastPosX = x;
    lastPosY = y;

    if (rotateFlag || (event->buttons() & Qt::LeftButton)) { // Rotate scene around a center point.
        float theta_y = 2.0 * dy / M_PI * 180.0f;
        float theta_x = 2.0 * dx / M_PI * 180.0f;

        QQuaternion revQ = camrot.conjugate();
        QQuaternion newrot = QQuaternion::fromAxisAndAngle(lookUp, theta_x);
        revQ = newrot * revQ;

        QVector3D side = revQ.rotatedVector(QVector3D(1,0,0));
        QQuaternion newrot2 = QQuaternion::fromAxisAndAngle(side, theta_y);
        revQ = newrot2 * revQ;
        revQ.normalize();

        camrot = revQ.conjugate().normalized();

        eye = newrot.rotatedVector(eye - lookCenter) + lookCenter;
        eye = newrot2.rotatedVector(eye - lookCenter) + lookCenter;
    }

    if (scaleFlag || (event->buttons() & Qt::MidButton)) { // Scale the scene.
        float factor = dx + dy;
        factor = exp(2.0 * factor);
        factor = (factor - 1.0) / factor;
        QVector3D translation = (lookCenter - eye) * factor;
        eye += translation;
    }

    if (translateFlag || (event->buttons() & Qt::RightButton)) { // Translate the scene.
        QQuaternion revQ = camrot.conjugate().normalized();
        QVector3D side = revQ.rotatedVector(QVector3D(1,0,0));
        QVector3D upVector = revQ.rotatedVector(QVector3D(0,1,0));

        float length = lookCenter.distanceToPoint(eye) * tanf(yfov * M_PI / 180.0f);
        QVector3D translation = -((side * (length * dx)) + (upVector * (length * dy) ));
        eye += translation;
        lookCenter += translation;
    }
    event->accept();
}


// toggle only the rotate flag
void GLview::toggleRotate() {
    if(mesh == NULL) return;
    translateFlag = scaleFlag = false;
    rotateFlag = !rotateFlag;
    setMouseTracking(rotateFlag);
    lastPosFlag = false;
}


// toggle only the scale flag
void GLview::toggleScale() {
    if(mesh == NULL) return;
    translateFlag = rotateFlag = false;
    scaleFlag = !scaleFlag;
    setMouseTracking(scaleFlag);
    lastPosFlag = false;
}


// toggle only the translate flag
void GLview::toggleTranslate() {
    if(mesh == NULL) return;
    rotateFlag = scaleFlag = false;
    translateFlag = !translateFlag;
    setMouseTracking(translateFlag);
    lastPosFlag = false;
}


void GLview::toggleLightMotion() {
    if(mesh == NULL) return;
    lightMotionFlag = !lightMotionFlag;
}


void GLview::updateGLview(float dt) {
    if(lightMotionFlag) {
        QQuaternion q1 = QQuaternion::fromAxisAndAngle(lookUp, dt * 30);
        LightPosition = q1.rotatedVector(LightPosition);

        QVector3D latVec = (LightPosition - lookCenter).normalized();
        QVector3D rotAxis = QVector3D::crossProduct(lookUp, latVec).normalized();
        QQuaternion q2 = QQuaternion::fromAxisAndAngle(rotAxis, dt * latitude_velocity);
        QVector3D LightPosition2 = q2.rotatedVector(LightPosition);

        QVector3D latVec2 = (LightPosition2 - lookCenter).normalized();
        float latAngle2 = acos(QVector3D::dotProduct(latVec2, lookUp.normalized() )) * 180.0 / M_PI;

        if(latAngle2 > 90 || latAngle2 < 10) {
            latitude_velocity = -latitude_velocity;
        }
        else {
            LightPosition = LightPosition2;
        }
    }
}


void GLview::timerEvent(QTimerEvent *) {
    if(!elapsed_time.isValid()) {
        elapsed_time.restart();
        return;
    } // Skip first udpate.
    qint64 nanoSec = elapsed_time.nsecsElapsed();
    elapsed_time.restart();

    double dt = 0.01;
    double frameTime = double(nanoSec) * 1e-9;

    timeAccumulator += frameTime;
    while ( timeAccumulator >= dt ) {
        updateGLview(dt);  totalTime += dt;  timeAccumulator -= dt;
    }
    update();
}
