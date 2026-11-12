// STL file load code from DownJ 2020.11.17
#ifndef STLloader_HPP
#define STLloader_HPP

#include <fstream>
#include <string>
#include <iostream>
#include <math.h>
#include <string.h>

using namespace std;

#ifndef EIGEN   // include eigen
#define EIGEN
#include "Eigen/Dense"
using namespace Eigen;

typedef Matrix<float, 4, 4> Transformation;
#endif

class StlLoader
{
private:
  // data size of STL file
  int num_point_;
  int num_vertex_;

  int** verticies_;			// vertices of STL file
  float** points_;         // point vectors of STL file
  float** normal_;			// normal vectors of STL file

  float** T_;

  bool isBinarySTL(char *);
  int set_UniquePoint(float x, float y, float z);

public:
  StlLoader();

  void get_Triangle(int idx, float* p1, float* p2, float* p3, float* normal);      // get vertices and normal vector in index idx.
  void STLImport(const string & _fileName, Transformation T);

  // get the data of STL file.
  inline int get_Npoint(){ return num_point_; };
  inline int get_Nvertex(){ return num_vertex_; };

  inline void get_Point(int idx, float& x, float& y, float& z) {
    x = points_[idx][0];  y = points_[idx][1];  z = points_[idx][2];
  };

  inline void get_Point(int idx, int i, float& x, float& y, float& z) {
    x = points_[verticies_[idx][i]][0];   y = points_[verticies_[idx][i]][1];   z = points_[verticies_[idx][i]][2];
  };

  inline void get_Normal(int idx, float &x, float &y, float &z) {
    x = normal_[idx][0];   y = normal_[idx][1];   z = normal_[idx][2];
  };

  inline void get_CenterPoint(int idx, float& x, float& y, float& z) {
    x = (points_[verticies_[idx][0]][0] + points_[verticies_[idx][1]][0] + points_[verticies_[idx][2]][0]) / 3;
    y = (points_[verticies_[idx][0]][1] + points_[verticies_[idx][1]][1] + points_[verticies_[idx][2]][1]) / 3;
    z = (points_[verticies_[idx][0]][2] + points_[verticies_[idx][1]][2] + points_[verticies_[idx][2]][2]) / 3;
  };

  inline void get_TriangleIndex(int idx, int& f1, int& f2, int& f3) {
    f1 = verticies_[idx][0];   f2 = verticies_[idx][1];   f3 = verticies_[idx][2];
  };
};



#endif // STLloader_HPP

