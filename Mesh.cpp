#include <iostream>
#include <fstream>
#include <iomanip>

#include "Mesh.hpp"

using namespace std;


void Mesh::add_face(const vector<int> &cur_vert, int mtl_idx) {
  vector<int> cur_vt(cur_vert.size(), -1);
  vector<int> cur_vn(cur_vert.size(), -1);
  add_face(cur_vert, cur_vt, cur_vn, mtl_idx);
}

void Mesh::add_face(int v0, int v1, int v2, int v3, int mtl_idx) {
  vector<int> faces(4, 0);
  faces[0] = v0; faces[1] = v1;  faces[2] = v2; faces[3] = v3;
  add_face(faces, mtl_idx);
}


void Mesh::add_face(const vector<int> &cur_vert, const vector<int> &cur_vt, const vector<int> &cur_vn, int mtl_idx) {
  materials[mtl_idx].size += cur_vert.size() - 2;
  if(cur_vert.size() > 3) {
    // If number of edges in face is greater than 3,
    // decompose into triangles as a triangle fan.
    int v0 = cur_vert[0], v1 = cur_vert[1], v2 = cur_vert[2];
    int t0 = cur_vt[0], t1 = cur_vt[1], t2 = cur_vt[2];
    int n0 = cur_vn[0], n1 = cur_vn[1], n2 = cur_vn[2];    

    faces.push_back(Mesh_Face(v0, v1, v2,
			      n0, n1, n2,
			      t0, t2, t2, mtl_idx));
    // First face
    for( size_t i = 3; i < cur_vert.size(); i++ ) {
      v1 = v2; v2 = cur_vert[i];
      t1 = t2; t2 = cur_vt[i];
      n1 = n2; n2 = cur_vn[i];      
      faces.push_back(Mesh_Face(v0, v1, v2,
			      n0, n1, n2,				
				t0, t1, t2, mtl_idx));
    }
  }
  else if(cur_vert.size() == 3) { 
    faces.push_back(Mesh_Face(cur_vert[0], cur_vert[1], cur_vert[2],
			      cur_vn[0], cur_vn[1], cur_vn[2], 			      
			      cur_vt[0], cur_vt[1], cur_vt[2], 
			      mtl_idx));
  }
}

bool Mesh::load_obj(QString filename, QString dir) {
  QFile objfile(filename);
  if (!objfile.open(QIODevice::ReadOnly | QIODevice::Text)) { 
    return false; //error
  }
  QTextStream in(&objfile);
  long face_cnt = 0;

  long mtl_idx = 0;
  Material default_mat;
  materials.push_back(default_mat);
  
  while (!in.atEnd()) {
    QString line = in.readLine(); 
    line = line.trimmed();
    line = line.replace("\t", " ");

    QStringList tokens = line.trimmed().split(' ', QString::SkipEmptyParts);
    if(tokens.size() == 0) continue;
    
    if(tokens[0] == "v") {
      if(tokens.size() < 4) return false; // eror
      float x = tokens[1].toFloat();
      float y = tokens[2].toFloat();
      float z = tokens[3].toFloat();
      vertices.push_back(QVector3D(x,y,z));
    }
    if(tokens[0] == "vn") {
      if(tokens.size() < 4) return false; // eror
      float x = tokens[1].toFloat();
      float y = tokens[2].toFloat();
      float z = tokens[3].toFloat();
      normals.push_back(QVector3D(x,y,z));
    }
    if(tokens[0] == "vt") {
      if(tokens.size() < 3) return false;
      float u = tokens[1].toFloat();
      float v = tokens[2].toFloat();
      texCoords.push_back(QVector2D(u,v));
    }
    if(tokens[0] == "f") {
      vector<int> cur_vert, cur_vt, cur_vn;
      for(int i = 1; i < tokens.size(); i++) {
        QStringList indexes = tokens[i].split("/");
        if(indexes.size() >= 1) {
          if(indexes[0].toLong() < 0) {  cur_vert.push_back(vertices.size() + indexes[0].toLong()); }
          else { cur_vert.push_back(indexes[0].toLong() - 1); }

          // Add texture index, if present.
          if(indexes.size() >= 2) {
            if(indexes[1] == "") cur_vt.push_back(-1);
            else {
              if(indexes[1].toLong() < 0) { cur_vt.push_back(texCoords.size() + indexes[1].toLong()); }
              else { cur_vt.push_back(indexes[1].toLong() - 1); }
            }
          }
          else cur_vt.push_back(-1);

          // Add normal index, if present.
          if(indexes.size() >= 3) {
            if(indexes[2] == "") cur_vn.push_back(-1);
            else {
              if(indexes[2].toLong() < 0) { cur_vn.push_back(normals.size() + indexes[2].toLong()); }
              else { cur_vn.push_back(indexes[2].toLong() - 1);}
            }
          }
          else cur_vn.push_back(-1);

        }
      }
      face_cnt++;
      add_face(cur_vert, cur_vt, cur_vn, mtl_idx);
    }    
    if(tokens[0] == "mtllib") {
      QStringList tokens = line.split(' ', QString::SkipEmptyParts);
      if(tokens.size() < 2) continue;
      QString mtllib_file = tokens[1];
      mtllib_file = dir + "/" + mtllib_file;
      bool ret = load_mtl(mtllib_file, dir);
      if(!ret) return false;
    }
    if(tokens[0] == "usemtl") {
      if(tokens.size() < 2) return false;
      QString mtl_name = tokens[1];
      for(long idx = 0; idx < (long)materials.size(); idx++) {
        if(materials[idx].name == mtl_name) {
          mtl_idx = idx;
          break;
        }
      }
    }
  }
  cout << "materials.size()=" << materials.size() << endl;
  cout << "face_cnt=" << face_cnt << endl;
  cout << "faces.size()=" << faces.size() << endl;
  cout << "vertices.size()=" << vertices.size() << endl;
  cout << "texCoords.size()=" << texCoords.size() << endl;
  rebuild_adj();
  compute_face_normals();
  // Only computed vertex normals if absent in obj file
  if(normals.size() == 0) compute_vertex_normals();
  recenter();
  cout << "normals.size()=" << normals.size() << endl;
  return true;
}

void Mesh::make_AABB(const QVector3D &A, const QVector3D &B) {
  Material default_mat;
  //default_mat.Ns = 100;
  default_mat.Ka = QVector3D(0.0, 0.1, 0.1);
  default_mat.Kd = QVector3D(0.0, 0.5, 0.5);
  default_mat.Ks = QVector3D(0,0,0);
  materials.push_back(default_mat);
  
  int mtlidx = 0;
  vertices.resize(8);

  vertices[0] = QVector3D(A[0], A[1], A[2]);
  vertices[1] = QVector3D(A[0], B[1], A[2]);
  vertices[2] = QVector3D(B[0], B[1], A[2]);
  vertices[3] = QVector3D(B[0], A[1], A[2]);

  vertices[4] = QVector3D(A[0], A[1], B[2]);
  vertices[5] = QVector3D(A[0], B[1], B[2]);
  vertices[6] = QVector3D(B[0], B[1], B[2]);
  vertices[7] = QVector3D(B[0], A[1], B[2]);

  add_face(0,1,2,3, mtlidx);
  add_face(7,6,5,4, mtlidx);
  add_face(5,6,2,1, mtlidx); 
  add_face(7,4,0,3, mtlidx); 
  add_face(1,0,4,5, mtlidx);
  add_face(6,7,3,2, mtlidx);
  rebuild_adj();
  compute_face_normals();
  compute_vertex_normals();
}

void Mesh::recenter() {
    if( vertices.size() < 1) return;
    QVector3D maxPoint = vertices[0];
    QVector3D minPoint = vertices[0];

    // Find the AABB
    for( uint i = 0; i < vertices.size(); ++i ) {
        QVector3D & point = vertices[i];
        if( point[0] > maxPoint[0] ) maxPoint[0] = point[0];
        if( point[1] > maxPoint[1] ) maxPoint[1] = point[1];
        if( point[2] > maxPoint[2] ) maxPoint[2] = point[2];
        if( point[0] < minPoint[0] ) minPoint[0] = point[0];
        if( point[1] < minPoint[1] ) minPoint[1] = point[1];
        if( point[2] < minPoint[2] ) minPoint[2] = point[2];
    }

    // Center of the AABB
    QVector3D center = QVector3D( (maxPoint[0] + minPoint[0]) / 2.0f,
                        (maxPoint[1] + minPoint[1]) / 2.0f,
                        (maxPoint[2] + minPoint[2]) / 2.0f );

    // Translate center of the AABB to the origin
    for( uint i = 0; i < vertices.size(); ++i ) {
        QVector3D & point = vertices[i];
        point = point - center;
    }
}

void Mesh::get_AABB(QVector3D &maxPoint, QVector3D &minPoint) {
    if( vertices.size() < 1) return;
    maxPoint = vertices[0];
    minPoint = vertices[0];

    // Find the AABB
    for( uint i = 0; i < vertices.size(); ++i ) {
        QVector3D & point = vertices[i];
        if( point[0] > maxPoint[0] ) maxPoint[0] = point[0];
        if( point[1] > maxPoint[1] ) maxPoint[1] = point[1];
        if( point[2] > maxPoint[2] ) maxPoint[2] = point[2];
        if( point[0] < minPoint[0] ) minPoint[0] = point[0];
        if( point[1] < minPoint[1] ) minPoint[1] = point[1];
        if( point[2] < minPoint[2] ) minPoint[2] = point[2];
    }

}


bool Mesh::load_mtl(QString filename, QString dir) {
  QFile objfile(filename);
  if (!objfile.open(QIODevice::ReadOnly | QIODevice::Text)) { 
    return false; //error
  }
  QTextStream in(&objfile);
  Material mat;
  while (!in.atEnd()) {
    QString line = in.readLine(); 
    line = line.trimmed();
    QStringList tokens = line.split(' ', QString::SkipEmptyParts);
    if(tokens.size() == 0) continue;
    if(tokens[0] == "newmtl") {
      if(tokens.size() < 2) return false;
      if(mat.name != "") {
        materials.push_back(mat);
      }
      mat.name = tokens[1];
    }
    if(tokens[0] == "Ka") {
      if(tokens.size() < 4) return false;
      mat.Ka[0] = tokens[1].toFloat();
      mat.Ka[1] = tokens[2].toFloat();
      mat.Ka[2] = tokens[3].toFloat();
    }
    if(tokens[0] == "Kd") {
      if(tokens.size() < 4) return false;
      mat.Kd[0] = tokens[1].toFloat();
      mat.Kd[1] = tokens[2].toFloat();
      mat.Kd[2] = tokens[3].toFloat();
    }
    if(tokens[0] == "Ks") {
      if(tokens.size() < 4) return false;
      mat.Ks[0] = tokens[1].toFloat();
      mat.Ks[1] = tokens[2].toFloat();
      mat.Ks[2] = tokens[3].toFloat();
    }
    if(tokens[0] == "Ns") {
      if(tokens.size() < 2) return false;
      mat.Ns = tokens[1].toFloat();
    }
    if(tokens[0] == "map_Kd") {
      if(tokens.size() < 2) return false;
      QString texture_file = tokens[1];
      texture_file = dir + "/" + texture_file;
      cout << "loading " << texture_file.toStdString() << endl;
      mat.map_Kd = new QOpenGLTexture(QImage(texture_file).mirrored());
      mat.map_Kd->setMinMagFilters(QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear);
      mat.map_Kd->generateMipMaps();
      mat.is_texture = true;
      cout << "*DONE*" << endl;
    }
  }
  if(mat.name != "") materials.push_back(mat);
  return true;
}

void Mesh::rebuild_adj() {
  // Compute how much space is needed for the face list on each vertex.
  vector<int> fadjcnt(vertices.size(), 0);
  for(size_t fidx = 0; fidx < faces.size(); fidx++) {
    for(int i = 0; i < 3; i++) fadjcnt.at(faces[fidx].vert[i])++;
  }

  for(size_t vidx = 0; vidx < vertices.size(); vidx++) 
    if(fadjcnt[vidx] == 0) { cerr << "no adjacent face for vertex" << endl;  }

  // Reserve space for each face list.
  facelist.resize(vertices.size());

  // Clear, if any are present.
  for(size_t vidx = 0; vidx < facelist.size(); vidx++) facelist[vidx].clear();
  for(size_t vidx = 0; vidx < facelist.size(); vidx++) facelist[vidx].reserve(fadjcnt[vidx]); 

  // Populate each face list.
  for(int fidx = 0; fidx < (int)faces.size(); fidx++) {
    for(int i = 0; i < 3; i++) facelist[faces[fidx].vert[i]].push_back(fidx);
  }

}

inline QVector3D ComputeNormal(QVector3D &p0, QVector3D &p1, QVector3D &p2) {
  QVector3D v1 = p1 - p0, v2 = p2 - p0;
  QVector3D normal = QVector3D::crossProduct(v1, v2);
  normal.normalize();
  return normal;
}
// Compute face normals using vertex indicies.
void Mesh::compute_face_normals() {
  for(int fidx = 0; fidx < (int)faces.size(); fidx++) {
    Mesh_Face &F = faces[fidx];
    QVector3D &v0 = vertices[F.vert[0]];
    QVector3D &v1 = vertices[F.vert[1]];
    QVector3D &v2 = vertices[F.vert[2]];
    faces[fidx].normal = ComputeNormal(v0, v1, v2);
  }
}


// Computes vertex normal as the mean of the neighboring face normals.
void Mesh::compute_vertex_normals() {
  // Compute vertex normals. 
  normals.reserve(vertices.size());
  normals.resize(vertices.size());
  for(size_t vidx = 0; vidx < vertices.size(); vidx++) {
    vector<long> &adj = facelist[vidx]; // Get face adjacencies.
    QVector3D vnormal(0,0,0); 
    for(size_t a = 0; a < adj.size(); a++) vnormal += faces[adj[a]].normal;
    float Z = (float)adj.size();
    normals[vidx] = (1.0f / Z) * vnormal;
    normals[vidx].normalize();
  }
}


void Mesh::storeVBO() {
  vector<QVector3D> tri_vert;
  vector<QVector3D> tri_norm;
  vector<QVector2D> tri_tex;
  for(long mtl_idx = 0; mtl_idx < (long)materials.size(); mtl_idx++) {
    for(long f = 0; f < (long)faces.size(); f++) {
      if(mtl_idx != faces[f].mtl_idx) continue;
      tri_vert.push_back(vertices.at(faces[f].vert[0]));
      tri_vert.push_back(vertices.at(faces[f].vert[1]));
      tri_vert.push_back(vertices.at(faces[f].vert[2]));

      for(int d = 0; d < 3; d++) {
        if(faces[f].vnidx[d] >= 0) {
          tri_norm.push_back(normals.at(faces[f].vnidx[d]));
        }
        else {
          tri_norm.push_back(normals.at(faces[f].vert[d]));
        }
      }

      if(faces[f].vt[0] >= 0) tri_tex.push_back(texCoords.at(faces[f].vt[0]));
      else tri_tex.push_back(QVector2D(0,0));

      if(faces[f].vt[1] >= 0) tri_tex.push_back(texCoords.at(faces[f].vt[1]));
      else tri_tex.push_back(QVector2D(0,0));
      
      if(faces[f].vt[2] >= 0) tri_tex.push_back(texCoords.at(faces[f].vt[2]));
      else tri_tex.push_back(QVector2D(0,0));


    }
  }

  vertexBuffer.create();
  vertexBuffer.setUsagePattern( QOpenGLBuffer::StaticDraw );
  vertexBuffer.bind();
  vertexBuffer.allocate(&tri_vert[0] , sizeof( QVector3D ) * tri_vert.size());

  normalBuffer.create();
  normalBuffer.setUsagePattern( QOpenGLBuffer::StaticDraw );
  normalBuffer.bind();
  normalBuffer.allocate(&tri_norm[0] , sizeof( QVector3D ) * tri_norm.size());    

  texCoordBuffer.create();
  texCoordBuffer.setUsagePattern( QOpenGLBuffer::StaticDraw );
  texCoordBuffer.bind();
  texCoordBuffer.allocate(&tri_tex[0] , sizeof( QVector2D ) * tri_tex.size());

}

