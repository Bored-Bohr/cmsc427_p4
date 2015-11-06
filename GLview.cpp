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
//    mesh = NULL;
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
//    if(mesh != NULL) delete mesh;
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

    if (meshList.size()==0){
      QVector3D pos0(0, 0, 0);
//      QQuaternion ori0 = QQuaternion.fromEulerAngles(0, 0, 0);
      QVector3D pos1;
      QQuaternion ori1;
      randCoordOrient(pos1, ori1);
      positions.push_back(pos0);
      orientations.push_back(QQuaternion::fromEulerAngles(0.0, 0.0, 0.0));
      newmesh->position = 0;
      positions.push_back(pos1);
      orientations.push_back(ori1);
    } else {
      QVector3D pos;
      QQuaternion ori;
      randCoordOrient(pos, ori);
      positions.push_back(pos);
      orientations.push_back(ori);
      newmesh->position = positions.size() - 1;
    }

    meshList.push_back(newmesh);
    meshList.at(meshList.size()-1)->storeVBO();

    doneCurrent();
    return true;
}

void GLview::randCoordOrient(QVector3D &position, QQuaternion &orientation) {
    float delta_x = (float)rand()/(float)(RAND_MAX/30) - 15;
    float delta_y = (float)rand()/(float)(RAND_MAX/30) - 15;
    float delta_z = (float)rand()/(float)(RAND_MAX/30) - 15;

    float theta_x = (float)rand()/(float)(RAND_MAX/360);
    float theta_y = (float)rand()/(float)(RAND_MAX/180);
    float theta_z = (float)rand()/(float)(RAND_MAX/360);

    orientation = QQuaternion::fromEulerAngles(theta_x, theta_y, theta_z);

    position = QVector3D(delta_x, delta_y, delta_z);
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
    if ( !prepareShaderProgram(phong_shader,  ":/texture.vsh", ":/texture.fsh" ) ) return;

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
    // comment out for windows
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
void GLview::initLightCameraGL(Mesh* mesh, long i) {
    QMatrix4x4 model, view, projection;
    // scale down the object if they are too large
    float largestDimension = mesh->dimensions.x();
    if (mesh->dimensions.y() > largestDimension) largestDimension = mesh->dimensions.y();
    if (mesh->dimensions.z() > largestDimension) largestDimension = mesh->dimensions.z();
    if (largestDimension > 5) model.scale(5.0 / largestDimension);

    float t = fmod(totalTime, 5.0) / 5.0;
    long pos_size = positions.size();
    long idx = i + (long(totalTime / 5) % pos_size);
    if (idx >= pos_size) idx -= pos_size;
    long idx2 = idx + 1;
    if (idx2 >= pos_size) idx2 -= pos_size;
    QQuaternion orientation = QQuaternion::slerp(orientations[idx],orientations[idx2],t);
    QVector3D position = QQuaternion::slerp(QQuaternion(1, positions[idx]), QQuaternion(1, positions[idx2]), t).vector();

    model.rotate(orientation);
    if (largestDimension > 5)
      model.translate(position * largestDimension / 5.0);
    else
      model.translate(position);

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
    if(meshList.size() == 0) return;

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
    for (size_t i =0; i <meshList.size();i++){
      Mesh *mesh = meshList.at(i);
      initLightCameraGL(mesh, i); // Update lighting and camara position.

//      QVector3D diff = positions.at(i+1) - positions.at(i);
//      mesh->position.setX(mesh->position.x()+0.002*diff.x());
//      mesh->position.setY(mesh->position.y()+0.002*diff.y());
//      mesh->position.setZ(mesh->position.z()+0.002*diff.z());

      mesh->vertexBuffer.bind();
      phong_shader.setAttributeBuffer( "VertexPosition", GL_FLOAT, 0, 3 );
      phong_shader.enableAttributeArray( "VertexPosition" );

      mesh->normalBuffer.bind();
      phong_shader.setAttributeBuffer( "VertexNormal", GL_FLOAT, 0, 3 );
      phong_shader.enableAttributeArray( "VertexNormal" );

      mesh->texCoordBuffer.bind();
      phong_shader.setAttributeBuffer( "VertexTexCoord", GL_FLOAT, 0, 2 );
      phong_shader.enableAttributeArray( "VertexTexCoord" );

      long total_drawn = 0;
      for(long mtl_idx = 0; mtl_idx < (long)mesh->materials.size(); mtl_idx++) {
        phong_shader.setUniformValue("Kd", mesh->materials[mtl_idx].Kd);
        phong_shader.setUniformValue("Ks", mesh->materials[mtl_idx].Ks);
        phong_shader.setUniformValue("Ka", mesh->materials[mtl_idx].Ka);
        phong_shader.setUniformValue("Shininess", mesh->materials[mtl_idx].Ns);
        phong_shader.setUniformValue("IsTexture", mesh->materials[mtl_idx].is_texture);
        if(mesh->materials[mtl_idx].is_texture) {
          mesh->materials[mtl_idx].map_Kd->bind(0);
          phong_shader.setUniformValue("Tex1", GLuint(0));
        }

        // Hint: texture unit 0 is free for your textures. I've bound the shadow map to texture unit 1.
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, depthTex);

        glDrawArrays( GL_TRIANGLES, total_drawn, 3*mesh->materials[mtl_idx].size );
        total_drawn += 3*mesh->materials[mtl_idx].size;
      }
    }
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
    if(meshList.empty()) return;
    float px = event->x(), py = event->y();
    float x = 2.0 * (px + 0.5) / float(width())  - 1.0,  y = -(2.0 * (py + 0.5) / float(height()) - 1.0);
    lastPosX = x; lastPosY = y;
    lastPosFlag = true;
    event->accept();
}

// Update camera position on mouse movement. DO NOT MODIFY.
void GLview::mouseMoveEvent(QMouseEvent *event) {
    if(meshList.empty()) return;

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
    if(meshList.empty()) return;
    translateFlag = scaleFlag = false;
    rotateFlag = !rotateFlag;
    setMouseTracking(rotateFlag);
    lastPosFlag = false;
}


// toggle only the scale flag
void GLview::toggleScale() {
    if(meshList.empty()) return;
    translateFlag = rotateFlag = false;
    scaleFlag = !scaleFlag;
    setMouseTracking(scaleFlag);
    lastPosFlag = false;
}


// toggle only the translate flag
void GLview::toggleTranslate() {
    if(meshList.empty()) return;
    rotateFlag = scaleFlag = false;
    translateFlag = !translateFlag;
    setMouseTracking(translateFlag);
    lastPosFlag = false;
}


void GLview::toggleLightMotion() {
    if(meshList.empty()) return;
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

    if (meshList.size()>1)
        return;
    eye = QVector3D(-3,3,3); lookCenter = QVector3D(-2,-2,0); lookUp = QVector3D(0,0,1);
    e = QVector3D(-3,3,3), lc= QVector3D(1,1,0), lu = QVector3D(0,0,1);
    QMatrix4x4 v1,v2;
    v1.lookAt(eye, lookCenter, lookUp);
    Matrix2Quaternion(camrot, v1);
    v2.lookAt(e,lc,lu);
    Matrix2Quaternion(camrot2, v2);
    float t = fmod(totalTime, 2.0);
    if (t > 1) t = 2 - t;
    camrot = camrot.slerp(camrot,camrot2,t);
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
