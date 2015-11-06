#ifndef __MESH_HPP__

#include <QtGui>
#include <QtOpenGL>

using namespace std;


// Material specification for phong shading.
struct Material {
  Material() { 
    // Default material values. 
    Ns = 100;
    is_texture = false; map_Kd = NULL;
    Kd = QVector3D(0.8, 0.8, 0.8);
    Ks = QVector3D(0.95, 0.95, 0.95);
    Ka = QVector3D(0.1, 0.1, 0.1);
    size = 0;
  }
  QString name;
  QVector3D Ka, Kd, Ks;
  float Ns;
  bool is_texture;
  long size;
  QOpenGLTexture *map_Kd;
};

 
struct Mesh_Face {
  Mesh_Face() {
    vert[0] = vert[1] = vert[2] = -1;
    vnidx[0] = vnidx[1] = vnidx[2] = -1;
    mtl_idx = -1;}
  Mesh_Face(long v0, long v1, long v2, 
	    long n0, long n1, long n2,
	    long t0, long t1, long t2,	    
	    long mtl_idx_set = -1) { 
    vert[0] = v0; vert[1] = v1; vert[2] = v2;
    mtl_idx = mtl_idx_set;
    vnidx[0] = n0; vnidx[1] = n1; vnidx[2] = n2;    
    vt[0] = t0; vt[1] = t1; vt[2] = t2;
  }
  Mesh_Face(long v0, long v1, long v2, long mtl_idx_set) { 
    vert[0] = v0; vert[1] = v1; vert[2] = v2;
    vnidx[0] = vnidx[1] = vnidx[2] = -1;
    mtl_idx = mtl_idx_set;
    vt[0] = vt[1] = vt[2] = -1;
  }
  QVector3D normal; 
  long vert[3]; // indices (in the vertex array) of all vertices (mesh_vertex)
  long vnidx[3]; // indices (in the normal array)
  long vt[3]; // indicies in texture array.
  long mtl_idx; // index into materials array
};

struct Mesh {
  Mesh() {  }
  ~Mesh() {
    for(size_t i = 0; i < materials.size(); i++) {  if(materials[i].map_Kd != NULL) delete materials[i].map_Kd;   }
  }

  vector<QVector3D> vertices; // List of shared verticies.
  vector< vector<long> > facelist; // Adjacent face list for each vertex
  vector<QVector2D> texCoords; // Texture coordinates for each vertex.
  vector<QVector3D> normals; // Vertex normals.
  vector<Mesh_Face> faces; // Mesh faces.
  long position;
  QVector3D dimensions;

  vector<Material> materials;

  QOpenGLBuffer vertexBuffer, normalBuffer, texCoordBuffer, kdBuffer, ksBuffer, kaBuffer, shininessBuffer, isTextureBuffer;
  
  bool load_obj(QString filename, QString dir);
  bool load_mtl(QString filename, QString dir);

  void make_AABB(const QVector3D &A, const QVector3D &B);

  void rebuild_adj();
  void compute_face_normals();
  void compute_vertex_normals();
  void storeVBO();

  void recenter();
  void compute_dimensions();
  void get_AABB(QVector3D &maxPoint, QVector3D &minPoint);

  void add_face(const vector<int> &cur_vert, int mtl_idx);
  void add_face(int v0, int v1, int v2, int v3, int mtl_idx = 0);
  void add_face(const vector<int> &cur_vert, const vector<int> &cur_vt, const vector<int> &cur_vn, int mtl_idx);
};


#endif // __MESH_HPP__
