// reading a text file
#include <stdio.h>
#include <string.h>

#include <vector>
#include <cmath>
#include <time.h>
#include <math.h>
#include <sstream>
#include <iterator>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <lodepng.h>

#include <iostream>
#include <fstream>
#include <string>

#define PI 3.14159265 
inline float sqr(float x) { return x*x; }

using namespace std;
using namespace Eigen;

//****************************************************
// Some Classes and Structs
//****************************************************
struct RGB {
  float r;
  float g;
  float b;
};

struct Camera{
  Vector4f eye;
  Vector4f LL;
  Vector4f LR;
  Vector4f UL;
  Vector4f UR;
};

struct PointLight{
  Vector4f coord;
  RGB color;
  int falloff;
};

struct DirectLight{
  Vector4f coord;
  RGB color;
};

struct Material{
  RGB ambient;
  RGB diffuse;
  RGB specular;
  RGB reflective;
  float power;
};

class Sphere{
public:
  Sphere(Vector4f, float, Material, Matrix4f);
  Sphere();
  Vector4f center;
  float radius;
  Material mat;
  Matrix4f trans;
  Vector4f tempNormal;
  Vector4f tempPoint;
  float t;

};

Sphere::Sphere(Vector4f cen, float rad, Material material, Matrix4f transformationMatrix){
  center = cen;
  radius = rad;
  mat = material;
  trans = transformationMatrix;
  Vector4f tempNormal(0, 0, 0, 0);
  Vector4f tempPoint(0, 0, 0, 0);
  t = 0.0f;
}

Sphere::Sphere(){
}

class Triangle{
public:
  Triangle(Vector4f, Vector4f, Vector4f, Material, Matrix4f);
  Triangle(Vector4f, Vector4f, Vector4f, Material, Matrix4f, vector<Vector4f>);
  Triangle();
  Vector4f a;
  Vector4f b;
  Vector4f c;
  Material mat;
  Matrix4f trans;
  Vector4f tempNormal;
  Vector4f tempPoint;
  float t;
  vector<Vector4f> vertNorms;
};

Triangle::Triangle(Vector4f pointA, Vector4f pointB, Vector4f pointC, Material material, Matrix4f transformationMatrix){
  a = pointA;
  b = pointB;
  c = pointC;
  mat = material;
  trans = transformationMatrix;
  Vector4f tempNormal(0, 0, 0, 0);
  Vector4f tempPoint(0, 0, 0, 0);
  t = 0.0f;
  vector<Vector4f> vertNorms;
}

Triangle::Triangle(Vector4f pointA, Vector4f pointB, Vector4f pointC, Material material, Matrix4f transformationMatrix, vector<Vector4f> norms){
  a = pointA;
  b = pointB;
  c = pointC;
  mat = material;
  trans = transformationMatrix;
  Vector4f tempNormal(0, 0, 0, 0);
  Vector4f tempPoint(0, 0, 0, 0);
  t = 0.0f;
  vertNorms = norms;
}

Triangle::Triangle(){
}

class Ray{
public:
  Ray(Vector4f, Vector4f);
  Vector4f e;
  Vector4f d;
  float dot(Ray ray);
};

Ray::Ray(Vector4f start, Vector4f dir){
  e = start;
  d = dir.normalized();
}

float Ray::dot(Ray ray){
  return d.dot(ray.d);
}


//****************************************************
// Global Variables
//****************************************************
const float width = 1000;
const float height = 1000;
const float rlimit = 2;
Camera camera;
RGB gambient = {0.0f, 0.0f, 0.0f};
vector<Sphere> sphereArray;
vector<Triangle> triangleArray;
vector<PointLight> pLightArray;
vector<DirectLight> dLightArray;
Matrix4f currentTransformation;
RGB def = {0.0f, 0.0f, 0.0f};
Material material = {def, def, def, def, 0.0f};
vector<unsigned char> image;


//****************************************************
// Helper Functions
//****************************************************
bool intersect(Vector4f start, Vector4f dir){
  for (int i = 0; i < sphereArray.size(); i++){
    Sphere curr = sphereArray[i];

    Vector4f transformedStart = curr.trans.inverse() * (start + dir);
    Vector4f transformedRay = curr.trans.inverse() * dir;

    Vector4f v = (transformedStart - curr.center);

    float det = sqr(v.dot(transformedRay)) - (v.dot(v) - sqr(curr.radius));

    //no hit if det < 0
    if (det < -0.0001){
      continue;
    }
    float t1 = (-transformedRay.dot(v) + sqrt(det))/(transformedRay.dot(transformedRay));
    float t2 = (-transformedRay.dot(v) - sqrt(det))/(transformedRay.dot(transformedRay));
    //printf("t1: %f and t2: %f \n",t1, t2);
    if (t1 > 0.0001){
      return true;
    }
    if (t2 > 0.0001){
      return true;
    }
  }
  for (int z = 0; z < triangleArray.size(); z++){
      Triangle curr = triangleArray[z];

      Vector4f transformedRay = curr.trans.inverse() * dir;
      Vector4f transformedStart = curr.trans.inverse() * (start + dir);

      Vector4f vertA = curr.a;
      Vector4f vertB = curr.b;
      Vector4f vertC = curr.c;
      float a = vertA[0]-vertB[0];
      float b = vertA[1]-vertB[1];
      float c = vertA[2]-vertB[2];

      float d = vertA[0]-vertC[0];
      float e = vertA[1]-vertC[1];
      float f = vertA[2]-vertC[2];

      float g = transformedRay[0];
      float h = transformedRay[1];
      float i = transformedRay[2];

      float j = vertA[0]-transformedStart[0];
      float k = vertA[1]-transformedStart[1];
      float l = vertA[2]-transformedStart[2];

      Matrix3f A;
      A << a, d, g, 
           b, e, h,
           c, f, i;

      float comp1 = e*i - h*f;
      float comp2 = g*f - d*i;
      float comp3 = d*h - e*g;
      float comp4 = a*k - j*b;
      float comp5 = j*c - a*l;
      float comp6 = b*l - k*c;

      float M = a*(comp1) + b*(comp2) + c*(comp3);
      float beta = (j*(comp1) + k*(comp2) + l*(comp3))/M;
      float gamma = (i*(comp4) + h*(comp5) + g*(comp6))/M;
      if(beta > 0 && gamma > 0 && gamma + beta < 1) {
        float currT = -(f*(comp4) + e*(comp5) + d*(comp6))/M;
        if (currT > 0){
          return true;
        }
      }
    }
  return false;
}

void parseObjFile(char* objFile) {
  ifstream stream;
  stream.open(objFile, ifstream::in);

  vector<Vector4f> vertices;
  Vector4f dummyVert(0, 0, 0, 0);
  vertices.push_back(dummyVert); //since indexing starts from 1 for obj files

  vector<Vector4f> normals;
  normals.push_back(dummyVert);

  if(!stream.is_open()){
    cout << "Can't open Obj file" << endl;
  } else {
    string line;
    while(!stream.eof()) {
      vector<string> split;
      string buf2;
      getline(stream,line);
      stringstream ss(line);
      while (ss >> buf2) {
        split.push_back(buf2);
      }
      if(split.size() == 0) {
        continue;
      } else if (split[0] == "v"){
        //vertex

        float x = atof(split[1].c_str()); //.c_str returns pointer to char array
        float y = atof(split[2].c_str());
        float z = atof(split[3].c_str());
        Vector4f vert(x,y,z,1);
        vertices.push_back(vert);
        
      } else if (split[0] == "vn"){
        //normal

        float x = atof(split[1].c_str());
        float y = atof(split[2].c_str());
        float z = atof(split[3].c_str());
        Vector4f norm(x,y,z,0);
        norm.normalized();
        normals.push_back(norm);

      } else if (split[0] == "vt") {    //ignore textures
          continue; 

      } else if (split[0] == "f") {
        // for (int i = 0; i < vertices.size(); i++) {    //print all vertices
        //   cout << vertices[i] <<endl;
        // }
        if (split[1].find("//") == string::npos) {    //f v1 v2 v3 
          int a = atoi(split[1].c_str());
          int b = atoi(split[2].c_str());
          int c = atoi(split[3].c_str());
          Triangle tri = Triangle(vertices[a], vertices[b], vertices[c], material, currentTransformation);
          triangleArray.push_back(tri);

        } else if (split[1].find("//") != string::npos) {        //f v1//vn1 v2//vn2 v3//vn3
        //calculate triangle normal from the vertices' normals, interpolate across. each vert can have it's own 
          vector<Vector4f> tempCoords;
          vector<Vector4f> tempNorms;
          //printf("blah\n");
          for (int i = 1; i <= 3; i++) {
            string block = split[i].c_str();
            string v = block.substr(0, block.find("//"));
            string n = block.substr(block.find("//") + 2, block.length());

            int vertIndex = atoi(v.c_str());
            Vector4f vert = vertices[vertIndex];     //v1, v2, vn3            
            tempCoords.push_back(vert);

            int normIndex = atoi(n.c_str());
            Vector4f norm = normals[normIndex];      //vn1, vn2, vn3
            tempNorms.push_back(norm);
          }
          if (tempNorms.empty()) {
            Triangle tri = Triangle(tempCoords[0], tempCoords[1], tempCoords[2], material, currentTransformation);
            triangleArray.push_back(tri);
          } else {
            Triangle tri = Triangle(tempCoords[0], tempCoords[1], tempCoords[2], material, currentTransformation, tempNorms);
            triangleArray.push_back(tri);
          }                           
        }
        
      } else {  //unsupported or unknown operatore;
        continue;
      }
    }
  }
}

//****************************************************
// Draw takes care of selecting pixels and getting
// the viewing rays.
//****************************************************

void encodeOneStep(const char* filename, std::vector<unsigned char>& image, unsigned width, unsigned height){
    //Encode the image
  image.resize(4 * width * height);
  unsigned error = lodepng::encode(filename, image, width, height);
  //if there's an error, display it
  if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
}


RGB raytracer(Vector4f start, Vector4f ray, int rDepth){
  if (rDepth < rlimit){
    float ts = numeric_limits<float>::infinity();
    float tt = numeric_limits<float>::infinity();
    Sphere currBestSphere;
    Triangle currBestTriangle;
    Material bestMatt;
    Vector4f bestIntersect;
    Vector4f bestNormal;
    Matrix4f bestTransf;

    //finds first intersecting sphere
    for (int i = 0; i < sphereArray.size(); i++){
      Sphere curr = sphereArray[i];

      Vector4f transformedRay = (curr.trans.inverse() * ray);
      Vector4f transformedStart = (curr.trans.inverse() * start);

      Vector4f v = (transformedStart - curr.center);

      float det = sqr(transformedRay.dot(v)) - (transformedRay.dot(transformedRay))*(v.dot(v) - sqr(curr.radius));
      // printf("raytracer v.dot ray:%f v.dot v:%f sqr rad:%f \n", sqr(v.dot(transformedRay)), v.dot(v), sqr(curr.radius));

      //no hit if det < 0
      if (!(det < 0)){
        float t1 = (-(v.dot(transformedRay)) + sqrt(det))/(transformedRay.dot(transformedRay));
        float t2 = (-(v.dot(transformedRay)) - sqrt(det))/(transformedRay.dot(transformedRay));
        if (t1 > 0){
          if (t1 < ts){
            ts = t1;
            currBestSphere = curr;
          }
        }
        if (t2 > 0){
          if (t2 < ts){
            ts = t2;
            currBestSphere = curr;
          }
        }
        Vector4f intersect = transformedStart + ts * transformedRay;
        Vector4f normal = (intersect - curr.center);
        normal = curr.trans.inverse().transpose() * normal;
        normal[3] = 0;
        currBestSphere.tempNormal = normal.normalized();
        currBestSphere.tempPoint = start + ts * ray;
        currBestSphere.t = ts;
      }
    }

    //finds first intersecting triangle
    for (int z = 0; z < triangleArray.size(); z++){
      Triangle curr = triangleArray[z];

      Vector4f transformedRay = curr.trans.inverse() * ray;
      Vector4f transformedStart = curr.trans.inverse() * start;

      Vector4f vertA = curr.a;
      Vector4f vertB = curr.b;
      Vector4f vertC = curr.c;
      float a = vertA[0]-vertB[0];
      float b = vertA[1]-vertB[1];
      float c = vertA[2]-vertB[2];

      float d = vertA[0]-vertC[0];
      float e = vertA[1]-vertC[1];
      float f = vertA[2]-vertC[2];

      float g = transformedRay[0];
      float h = transformedRay[1];
      float i = transformedRay[2];

      float j = vertA[0]-transformedStart[0];
      float k = vertA[1]-transformedStart[1];
      float l = vertA[2]-transformedStart[2];

      Matrix3f A;
      A << a, d, g, 
           b, e, h,
           c, f, i;

      float comp1 = e*i - h*f;
      float comp2 = g*f - d*i;
      float comp3 = d*h - e*g;
      float comp4 = a*k - j*b;
      float comp5 = j*c - a*l;
      float comp6 = b*l - k*c;

      float M = a*(comp1) + b*(comp2) + c*(comp3);
      float beta = (j*(comp1) + k*(comp2) + l*(comp3))/M;
      float gamma = (i*(comp4) + h*(comp5) + g*(comp6))/M;
      if(beta > 0 && gamma > 0 && gamma + beta < 1) {
        float currT = -(f*(comp4) + e*(comp5) + d*(comp6))/M;
        if (currT < tt) {
          tt = currT;
          currBestTriangle = curr;
        }
      }
      currBestTriangle.tempPoint = start + tt * ray; //transformed ray?
      if (currBestTriangle.vertNorms.empty()) {  //flat shading
        Vector4f ba = currBestTriangle.b - currBestTriangle.a;
        Vector4f ca = currBestTriangle.c - currBestTriangle.a;
        Vector3f bac(ba[0], ba[1], ba[2]);
        Vector3f cac(ca[0], ca[1], ca[2]);
        Vector3f noc = bac.cross(cac);
        Vector4f normal(noc[0], noc[1], noc[2], 0); 
        currBestTriangle.tempNormal = normal.normalized();
        if (currBestTriangle.tempNormal.dot(ray) > 0){
          currBestTriangle.tempNormal = -currBestTriangle.tempNormal;
        }

      } else {    //smooth shaded obj triangle

        Vector4f intersect = currBestTriangle.tempPoint; 
        Vector4f normA = currBestTriangle.vertNorms[0];
        Vector4f normB = currBestTriangle.vertNorms[1];
        Vector4f normC = currBestTriangle.vertNorms[2];

        float alpha = 1 - beta - gamma;

        currBestTriangle.tempNormal = (alpha*normA + beta*normB + gamma*normC).normalized();
      }
    } //end loop over triangles
    //check if triangle or sphere was hit first
    //didn't hit anything

    // if (ts < numeric_limits<float>::infinity()){
    //   printf("%f \n", ts);
    // }
    if (ts == numeric_limits<float>::infinity() && tt == numeric_limits<float>::infinity()){
      RGB toRet = {0.0f, 0.0f, 0.0f};
      return toRet;

    } else if(tt < ts) {
      //hits triangle first
      bestMatt = currBestTriangle.mat;
      bestIntersect = currBestTriangle.tempPoint;
      bestNormal = currBestTriangle.tempNormal;

    }else if(ts < tt){
      //hits a sphere first
      bestMatt = currBestSphere.mat;
      bestIntersect = currBestSphere.tempPoint;
      bestNormal = currBestSphere.tempNormal;
    }

    //start shading
    float r, g, b, a;
    r = 0.0f;
    g = 0.0f;
    b = 0.0f;
    a = 1.0f;
  
    //ambient lighting
    r += gambient.r * bestMatt.ambient.r;
    g += gambient.g * bestMatt.ambient.g;
    b += gambient.b * bestMatt.ambient.b;

    //pointlights
    for (int k = 0; k < pLightArray.size(); k++) {
      PointLight light = pLightArray[k];

      //shading
      Vector4f dir = (light.coord - bestIntersect).normalized();


      //ambient term
      r += bestMatt.ambient.r * light.color.r;
      g += bestMatt.ambient.g * light.color.g;
      b += bestMatt.ambient.b * light.color.b;

      if (!intersect(bestIntersect, dir)){
      // Vector4f v1 = (currBestSphere.center - tStart);

      // float det1 = sqr(v1.dot(tRay)) - (v1.dot(v1) - sqr(currBestSphere.radius));
      // printf("det1:%f \n", det1);
      // printf("v.d:%f \n", v1.dot(tRay));

      // if (det1 > 0){
      //   printf("got here! \n");

        //diffuse term
        float dp = dir.dot(bestNormal);
        r += bestMatt.diffuse.r * (light.color.r * max(0.0f, dp));
        g += bestMatt.diffuse.g * (light.color.g * max(0.0f, dp));
        b += bestMatt.diffuse.b * (light.color.b * max(0.0f, dp));

        //specular term
        Vector4f reflected(bestNormal[0]*dp*2-dir[0], bestNormal[1]*dp*2-dir[1],bestNormal[2]*dp*2-dir[2], 0.0f);
        float spec = pow(max(0.0f, -ray.dot(reflected)), bestMatt.power); 

        r+= bestMatt.specular.r * (light.color.r * spec);
        g+= bestMatt.specular.g * (light.color.g * spec);
        b+= bestMatt.specular.b * (light.color.b * spec);
      }
    }//end point lights

    //handle all directional lights
    for (int k = 0; k < dLightArray.size(); k++) {

        DirectLight light = dLightArray[k];

        //ambient 
        r += bestMatt.ambient.r * light.color.r;
        g += bestMatt.ambient.g * light.color.g;
        b += bestMatt.ambient.b * light.color.b;

        if (!intersect(bestIntersect, -light.coord.normalized())) {

        //diffuse
        Vector4f dir = -light.coord.normalized();
        float dp = dir.dot(bestNormal);
        r += bestMatt.diffuse.r * (light.color.r * max(0.0f, dp));
        g += bestMatt.diffuse.g * (light.color.g * max(0.0f, dp));
        b += bestMatt.diffuse.b * (light.color.b * max(0.0f, dp));

        //specular
        Vector4f reflected(bestNormal[0]*dp*2-dir[0], bestNormal[1]*dp*2-dir[1],bestNormal[2]*dp*2-dir[2], 0.0f);
        float spec = pow(max(0.0f, -ray.dot(reflected)), bestMatt.power); 

        r+= bestMatt.specular.r * (light.color.r * spec);
        g+= bestMatt.specular.g * (light.color.g * spec);
        b+= bestMatt.specular.b * (light.color.b * spec);
      }
    } //end directional lights

    Vector4f reflectRay = ray - 2*bestNormal.dot(ray)*bestNormal;
    Vector4f dref = reflectRay.normalized();
    RGB reflections = raytracer(bestIntersect+dref, reflectRay, rDepth+1);
    r += bestMatt.reflective.r * reflections.r;
    g += bestMatt.reflective.g * reflections.g;
    b += bestMatt.reflective.b * reflections.b;

    r = min(r, 1.0f);
    g = min(g, 1.0f);
    b = min(b, 1.0f);

    RGB toRet = {r, g, b};
    return toRet;

  }
}

void Sampler() {
  Vector4f dwidth = (camera.LR - camera.LL) / width;
  Vector4f dheight = (camera.LL - camera.UL) / height;
  for (int i = 0; i < height; i++){
    for (int j = 0; j < width; j++){
      Vector4f viewPoint = camera.UL + float(i)*dheight + float(j)*dwidth;
      Vector4f viewingRay = viewPoint - camera.eye;
      RGB pixel = raytracer(camera.eye, viewingRay.normalized(), 0);
      int count = 4 * ((i * width) + j);
      image[count] = pixel.r * 255;
      image[count + 1] = pixel.g * 255;
      image[count + 2] = pixel.b * 255;
      image[count + 3] = 255.0f;
    }
  }
  int spheres = 0;
  for (int i = 0; i < sphereArray.size(); i++){
    spheres++;
  }
  const char* filename = "test.png";
  encodeOneStep(filename, image, width, height);

}

//****************************************************
// Main Function
//****************************************************
int main (int argc, char* argv[]) {
  //initializes the transformation matrix
  currentTransformation << 1, 0, 0, 0,
                           0, 1, 0, 0,
                           0, 0, 1, 0,
                           0, 0, 0, 1;

  //init image
  image.resize(4 * width * height);

  //Begin parsing input from input file
  string line;
  ifstream myfile (argv[1]);
  if (myfile.is_open()){

    while (getline(myfile,line)){
      char type[4];
      strncpy(type, line.c_str(), 3);
      type[3] = '\0';
      istringstream buf(line);
      char dummy[4];

      if (!strcmp(type, "cam")){
        float cam[15];
        buf >> dummy >> cam[0] >> cam[1] >> cam[2] >> cam[3] >> cam[4] >> cam[5] >> cam[6] >> cam[7] >> cam[8] >> cam[9] >> cam[10] >> cam[11] >> cam[12] >> cam[13] >> cam[14];
        Vector4f eye(cam[0], cam[1], cam[2], 1);
        Vector4f LL(cam[3], cam[4], cam[5], 1);
        Vector4f LR(cam[6], cam[7], cam[8], 1);
        Vector4f UL(cam[9], cam[10], cam[11], 1);
        Vector4f UR(cam[12], cam[13], cam[14], 1);
        camera.eye = eye;
        camera.LL = LL;
        camera.LR = LR;
        camera.UL = UL;
        camera.UR = UR;
      }else if(!strcmp(type, "sph")){
        float sph[4];
        buf >> dummy >> sph[0] >> sph[1] >> sph[2] >> sph[3];
        Vector4f center(sph[0], sph[1], sph[2], 1);
        Sphere sph1 = Sphere(center, sph[3], material, currentTransformation);
        sphereArray.push_back(sph1);

      }else if(!strcmp(type, "tri")){
        float tri[9];
        buf >> dummy >> tri[0] >> tri[1] >> tri[2] >> tri[3] >> tri[4] >> tri[5] >> tri[6] >> tri[7] >> tri[8];
        Vector4f pA(tri[0], tri[1], tri[2], 1);
        Vector4f pB(tri[3], tri[4], tri[5], 1);
        Vector4f pC(tri[6], tri[7], tri[8], 1);
        Triangle tri1 = Triangle(pA, pB, pC, material, currentTransformation);
        triangleArray.push_back(tri1);

      }else if(!strcmp(type, "obj")){
        string temp[2];
        buf >> temp[0] >> temp[1];
        
        char* objFile = new char[temp[1].size()+1];//.c_str();
        copy(temp[1].begin(), temp[1].end(), objFile);
        objFile[temp[1].size()] = '\0';
        printf("test file name is: %s \n", objFile);

        parseObjFile(objFile);

      }else if(!strcmp(type, "ltp")){
        float ltp[7];
        buf >> dummy >> ltp[0] >> ltp[1] >> ltp[2] >> ltp[3] >> ltp[4] >> ltp[5] >> ltp[6];
        Vector4f coord(ltp[0], ltp[1], ltp[2], 1.0);
        RGB color = {ltp[3], ltp[4], ltp[5]};
        PointLight pl = {coord, color, ltp[6]};
        pLightArray.push_back(pl);
      }else if(!strcmp(type, "ltd")){
        float ltd[6];
        buf >> dummy >> ltd[0] >> ltd[1] >> ltd[2] >> ltd[3] >> ltd[4] >> ltd[5];
        Vector4f coord(ltd[0], ltd[1], ltd[2], 0);
        RGB color = {ltd[3], ltd[4], ltd[5]};
        DirectLight dl = {coord, color};
        dLightArray.push_back(dl);
      }else if(!strcmp(type, "lta")){
        float lta[3];
        buf >> dummy >> lta[0] >> lta[1] >> lta[2];
        gambient.r = lta[0];
        gambient.g = lta[1];
        gambient.b = lta[2];
        
      }else if(!strcmp(type, "mat")){
        float mat[13];
        buf >> dummy >> mat[0] >> mat[1] >> mat[2] >> mat[3] >> mat[4] >> mat[5] >> mat[6] >> mat[7] >> mat[8] >> mat[9] >> mat[10] >> mat[11] >> mat[12];
        RGB amb = {mat[0], mat[1], mat[2]};
        RGB dif = {mat[3], mat[4], mat[5]};
        RGB spe = {mat[6], mat[7], mat[8]};
        RGB ref = {mat[10], mat[11], mat[12]};
        Material newMat = {amb, dif, spe, ref, mat[9]};
        material = newMat;
        
      }else if(!strcmp(type, "xft")){
        float xft[3];
        buf >> dummy >> xft[0] >> xft[1] >> xft[2];
        Matrix4f translation;
        translation << 1, 0, 0, xft[0],
                       0, 1, 0, xft[1],
                       0, 0, 1, xft[2],
                       0, 0, 0, 1;
        currentTransformation = currentTransformation * translation;
        
      }else if(!strcmp(type, "xfr")){
        float xfr[3];
        buf >> dummy >> xfr[0] >> xfr[1] >> xfr[2];

      }else if(!strcmp(type, "xfs")){
        float xfs[3];
        buf >> dummy >> xfs[0] >> xfs[1] >> xfs[2];
        Matrix4f scale;
        scale << xfs[0], 0, 0, 0,
                 0, xfs[1], 0, 0,
                 0, 0, xfs[2], 0,
                 0, 0, 0, 1;
        currentTransformation = currentTransformation * scale;
        
      }else if(!strcmp(type, "xfz")){
        currentTransformation << 1, 0, 0, 0,
                                 0, 1, 0, 0,
                                 0, 0, 1, 0,
                                 0, 0, 0, 1;
      }else{
        continue;
      }
    }
    myfile.close();
  }else{ 
    cout << "Unable to open file"; 
  }
  Sampler();
  return 0;
}