#include "stl_loader.hpp"

StlLoader::StlLoader(){
  num_point_  = 0;
  num_vertex_  = 0;
}

void StlLoader::STLImport(const string & fileName,  Transformation T){
  // Import a binary STL file.
  // Open the file for reading using an input fstream.

  ifstream ifs(fileName, ifstream::binary);

  // Get pointer to the associated buffer object.
  // rdbuf returns a streambuf object associated with the
  // input fstream object ifs.

  filebuf* pbuf = ifs.rdbuf();

  // Calculate the file's size.

  auto size = pbuf->pubseekoff(0, ifs.end);

  if(size==-1){
    std::cout << "StlLoader Error: can not find \"" << fileName << "\"\n";
    return;
  }

  // Set the position pointer to the beginning of the file.

  pbuf->pubseekpos(0);

  // Allocate memory to contain file data.

  char* buffer = new char[(size_t)size];

  // Get file data. sgetn grabs all the characters from the streambuf
  // object 'pbuf'. The return value of sgetn is the number of characters
  // obtained - ordinarily, this value should be checked for equality
  // against the number of characters requested.

  pbuf->sgetn(buffer, size);

  // Test to see if the file is binary.

  if (!isBinarySTL(buffer)) return;

  char * bufptr = buffer;

  bufptr += 80;  // Skip past the header.
  bufptr += 4;   // Skip past the number of triangles.

  ////////////////////////// variable initialization //////////////////////////
  num_vertex_ = (buffer + size - bufptr)/50;
  int i = 0, j = 0;

  verticies_ = new int* [num_vertex_];
  for (i = 0; i < num_vertex_; i++)  verticies_[i] = new int[3];
  
  normal_ = new float*[num_vertex_];
  for (i = 0; i < num_vertex_; i++)  normal_[i] = new float[3];
  
  points_ = new float* [3 * num_vertex_];
  for (i = 0; i < 3 * num_vertex_; i++)  points_[i] = new float[3];
  
  T_ = new float* [4];
  for (i = 0; i < 4; i++)
      T_[i] = new float[4];
  
  for(i = 0; i < 4; i++)  for(j = 0; j < 4; j++)   T_[i][j] = T(i,j);
  //////////////////////////////////////////////////////////////////////////////

  i = 0;
  while (bufptr < buffer + size) {

    float temp_normal[3];
    temp_normal[0] = *(float *)(bufptr);
    temp_normal[1] = *(float *)(bufptr + 4);
    temp_normal[2] = *(float *)(bufptr + 8);

    normal_[i][0] = T_[0][0] * temp_normal[0] + T_[0][1] * temp_normal[1] + T_[0][2] * temp_normal[2];
    normal_[i][1] = T_[1][0] * temp_normal[0] + T_[1][1] * temp_normal[1] + T_[1][2] * temp_normal[2];
    normal_[i][2] = T_[2][0] * temp_normal[0] + T_[2][1] * temp_normal[1] + T_[2][2] * temp_normal[2];
    bufptr += 12;
        
    verticies_[i][0] = set_UniquePoint(*(float*)(bufptr), *(float*)(bufptr + 4), *(float*)(bufptr + 8));
    bufptr += 12;

    verticies_[i][1] = set_UniquePoint(*(float*)(bufptr), *(float*)(bufptr + 4), *(float*)(bufptr + 8));
    bufptr += 12;

    verticies_[i][2] = set_UniquePoint(*(float*)(bufptr), *(float*)(bufptr + 4), *(float*)(bufptr + 8));
    bufptr += 12;

    if (i%1000 == 0)  std::cout << "iteration :" << i << "\n";
    
    bufptr += 2;
    i++;
  }

  cout << "STL file is imported sucessfully (vertex: " << num_vertex_ <<  ", point: " << num_point_ << ")\n";
  ifs.close();

  delete[] buffer;
}

bool StlLoader::isBinarySTL(char * buffer) {

  // Determine if a file is binary STL.

  bool bbinary = true;
  size_t spnsz, spnsz0;

  // Look for the first non-space character.

  spnsz = strspn(buffer, " ");

  char ctstr[6];  // Enough space to hold "solid\0" and "facet\0".

  // Copy the first five characters from the location of the first non-space
  // character to ctstr.

  strncpy(ctstr, &buffer[spnsz], 5);

  ctstr[5] = '\0';
  char csolid[] = "solid\0";

  // If this is an ASCII STL file, then the first string should be "solid".

  if (!strcmp(ctstr, csolid)) {
    // This file might be binary or text. To be certain we need to do a further test.
    // Read past the next new line. If this is a text file, there should be a newline.
    // The next token should be 'facet'.

    spnsz0 = 5 + spnsz;

    char * pch = strchr(&buffer[spnsz0], '\n');  // Look for the first instance of '\n'.
    // If pch is NULL then this is a binary STL file.
    if (pch) {
      pch++;

      spnsz = strspn(pch, " "); // Look for the first instance not of ' '.
      spnsz0 = spnsz;

      spnsz = strcspn(pch + spnsz0, " "); // Look for the first instance of ' '.

      if (spnsz == 5) {
        // Check for 'facet'.
        strncpy(ctstr, pch + spnsz0, 5);
        ctstr[5] = '\0';

        char cfacet[] = "facet\0";
        if (!strcmp(ctstr, cfacet)) {
          // This file is beyond reasonable doubt ASCII STL.
          bbinary = false;
        }
      }
    }
  }

  return(bbinary);
}


int StlLoader::set_UniquePoint(float x, float y, float z){
    float x2, y2, z2, x3, y3, z3;

    x2 = x - T_[0][3];
    y2 = y - T_[1][3];
    z2 = z - T_[2][3];

    // speed
//    x2 = x/1000 - T_[0][3];
//    y2 = y/1000 - T_[1][3];
//    z2 = z/1000 - T_[2][3];

    x3 = T_[0][0] * x2 + T_[0][1] * y2 + T_[0][2] * z2;
    y3 = T_[1][0] * x2 + T_[1][1] * y2 + T_[1][2] * z2;
    z3 = T_[2][0] * x2 + T_[2][1] * y2 + T_[2][2] * z2;


    for (int i = 0; i < num_point_; i++) {
        if ((x3 == points_[i][0]) && (y3 == points_[i][1]) && (z3 == points_[i][2])) return i;
    }

//    for(int i = 0; i < 3; i++)  points_[num_point_][i] = T_[0][i] * x2 + T_[1][i] * y2 + T_[2][i] * z2;
    points_[num_point_][0] = x3;
    points_[num_point_][1] = y3;
    points_[num_point_][2] = z3;
//    printf("%lf %lf %lf", x3, y3, z3);

    num_point_ ++;

    return num_point_ - 1;
}


void StlLoader::get_Triangle(int idx, float* p1, float* p2, float* p3, float* normal){
    for (int i = 0; i < 3; i++) {
      p1[i] = points_[verticies_[idx][0]][i];
      p2[i] = points_[verticies_[idx][1]][i];
      p3[i] = points_[verticies_[idx][2]][i];
      normal[i] = normal_[idx][i];
    }
}
