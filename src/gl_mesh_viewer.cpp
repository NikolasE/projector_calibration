/**
* File glfractal.cpp
* Brief Implementation of GLFractal
* This is a simple QGLWidget displaying an openGL wireframe box
*
* The OpenGL code is mostly borrowed from Brian Pauls "spin" example
* in the Mesa distribution.
* $ID$
*/

#include "projector_calibration/main_window.hpp"

using namespace projector_calibration;
using namespace std;


//#include "projector_calibration/glfractal.h"

/**
* Constructor that creates a GLFractal widget
*/
GL_Mesh_Viewer::GL_Mesh_Viewer( QWidget* parent)
: QGLWidget(parent)
{
 object = 0;
 initializeGL();
 show_texture = false;
 img_width = img_height = -1;
 draw_map = false;
 light_z_pos = -1;
}


GL_Mesh_Viewer::~GL_Mesh_Viewer()
{
 glDeleteLists( object, 1 );
}

void GL_Mesh_Viewer::LoadGLTextures() {

 ROS_INFO("Load texture");

 // Load Texture
 gl_Image *image1;

 // allocate space for texture
 image1 = (gl_Image *) malloc(sizeof(gl_Image));
 if (image1 == NULL) {
  printf("Error allocating space for image");
  exit(0);
 }



 // Create Texture
 glGenTextures(1, &texture[0]);
 glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

 glBindTexture(GL_TEXTURE_2D, texture[0]);   // 2d texture (x and y size)

 glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
 glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); // scale linearly when image smalled than texture


 texture_cv = cv::imread("imgs/text.bmp");
 // texture_cv = cv::imread("imgs/test.bmp");

 assert(texture_cv.type() == CV_8UC3);

 // ROS_INFO("texture: %i %i", texture_cv.cols, texture_cv.rows);
 // cv::cvtColor(texture_cv, texture_cv, CV_BGR2RGB);

 // 2d texture, level of detail 0 (normal), 3 components (red, green, blue), x size from image, y size from image,
 // border 0 (normal), rgb color data, unsigned byte data, and finally the data itself.
 glTexImage2D(GL_TEXTURE_2D, 0, 3, texture_cv.cols, texture_cv.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_cv.data);

 // cv::namedWindow("foo");
 // cv::imshow("foo", texture_cv);
 // cv::waitKey(10);

};



/// draw height lines as red GL_LINES
/**
*  @todo: color should depend on height
*/
void GL_Mesh_Viewer::drawHeightLines(){
 if (height_lines.size() == 0) return;

 glColor3f(1,0,0);
 glLineWidth(3.0);

 glBegin(GL_LINES);
 for (uint i =0; i<height_lines.size(); ++i){
  for (uint j=0; j<height_lines[i].size(); ++j){
   PointPair* pp = &height_lines[i][j];

   glVertex3f(pp->first[0],pp->first[1],pp->first[2]);
   glVertex3f(pp->second[0],pp->second[1],pp->second[2]);
  }
 }

 glEnd();

}


void GL_Mesh_Viewer::drawAnts(){

 // ROS_INFO("Drawing %zu ants", ants.size());

 if (!ants) return;

 for (std::map<int,Ant>::iterator it = ants->begin(); it != ants->end(); ++it){
  drawAnt(&it->second);
  drawPath(&it->second);
 }

}


/**
* @todo store cloud
* @param ant
*/
void GL_Mesh_Viewer::drawAnt(Ant* ant){

 // ROS_INFO("Drawing ant with id %i", ant->getId());

 if (ant->getState() == ANT_NOT_INITIALIZED){
  ROS_WARN("Can't show uninitialized ant");
  return;
 }


 // get 3d Position
 Cloud cloud;
 pcl::fromROSMsg(mesh->cloud, cloud);

 cv::Point pos = ant->getPosition();


 glLineWidth(10);

 // glPointSize(100);

 int cnt = 4;

 glBegin(GL_LINE_STRIP);

 for (int foo = -cnt; foo <= cnt; ++foo){

  int x = foo+pos.x;

  if (x<0 || x >= int(cloud.width)) continue;

  pcl_Point P = cloud.at(x, pos.y);

  if (foo == 0)
   glColor3f(0,1,0);
  else
   glColor3f(1,0,0);

  glVertex3f( P.x,P.y,P.z);
 }

 glEnd();

 glBegin(GL_LINE_STRIP);

 for (int foo = -cnt; foo <= cnt; ++foo){

  int y = foo+pos.y;

  if (y<0 || y >= int(cloud.height)) continue;

  pcl_Point P = cloud.at(pos.x, y);

  if (foo == 0)
   glColor3f(0,1,0);
  else
   glColor3f(1,0,0);

  glVertex3f( P.x,P.y,P.z);
 }

 glEnd();



 // glColor3f(1,0,0);
 // glVertex3f( P3.x+l,P3.y+l,P3.z+dz);
 //
 // glColor3f(1,0,0);
 // glVertex3f( P3.x,P3.y,P3.z+dz);
 //
 // glColor3f(1,0,0);
 // glVertex3f( P3.x-l,P3.y-l,P3.z+dz);
 //
 // glEnd();
 //
 // glBegin(GL_LINE_STRIP);
 //
 // glColor3f(1,0,0);
 // glVertex3f( P3.x+l,P3.y-l,P3.z+dz);
 //
 // glColor3f(1,0,0);
 // glVertex3f( P3.x,P3.y,P3.z+dz);
 //
 // glColor3f(1,0,0);
 // glVertex3f( P3.x-l,P3.y+l,P3.z+dz);
 //
 // glEnd();



 //  GLUquadricObj* Sphere = gluNewQuadric();
 //
 //  gluSphere(Sphere,0.02,20,20);
 //  gluDeleteQuadric(Sphere);
 //
 //  glPopMatrix();


}



/// Visualize Path
/**
* @todo: render as cylinder strip (gluCylinder)
*/
void GL_Mesh_Viewer::drawPath(Ant* ant){

 // ROS_INFO("draw path for ant %i", ant->getId());

 Cloud cloud;
 pcl::fromROSMsg(mesh->cloud, cloud);

 assert(cloud.width > 1 && cloud.height > 1);

 glColor3f(0,1,0);
 glLineWidth(5.0);

 // bool with_colors = (pathColors.size() == path.size());
 // cv::Vec3b* col;

 glBegin(GL_LINE_STRIP);

 int size = ant->getPathLenth();

 // ROS_INFO("Path length: %i", size);

 for (int i=0; i<size; ++i){

  cv::Point pos = ant->getPositionAt(i);

  pcl_Point p = cloud.at(pos.x, pos.y);

  //  ROS_INFO("Path point: %f %f %f", p.x,p.y,p.z);
  //  if (with_colors){
  //   col = &pathColors[i];
  //   glColor3f(col->val[0]/255.0, col->val[1]/255.0, col->val[2]/255.0);
  //  }

  glVertex3f(p.x,p.y,p.z);
 }

 glEnd();


}


/**
*
* @param triangles  list of triangles on surface with uv-coordinates for each corner
* @param uv_values  map of all uv-coordinates
* @param uv_inv_scale inverse of minimal geodesic distance
* @param center_id  id of center point of patch
*/
void GL_Mesh_Viewer::storeExpMapInfo(const std::vector<cv::Vec3i>& triangles, const cv::Mat& uv_values, float uv_inv_scale,int center_id){
 this->triangles = triangles;
 this->uv_values = uv_values;
 this->uv_inv_scale = uv_inv_scale;

 patch_center_id = center_id;

 // Cloud cloud;
 // pcl::fromROSMsg(mesh->cloud, cloud);
 // ROS_INFO("cloud: %zu", cloud.size());
 // this->patch_center.x = center_id%cloud.width;
 // this->patch_center.y = center_id/cloud.width;
 // ROS_INFO("centerid: %i, x: %i y: %i, width: %i", center_id, this->patch_center.x,this->patch_center.y,cloud.width);

}

void GL_Mesh_Viewer::removeExpMapInfo(){ this->triangles.clear();}



void GL_Mesh_Viewer::showExpMapTexture(){

 if (triangles.empty())
  return;


 assert(uv_values.type() == CV_32FC2);

 glEnable(GL_TEXTURE_2D);

 if (texture_cv.cols != 256)
  LoadGLTextures();

 Cloud cloud;
 pcl::fromROSMsg(mesh->cloud, cloud);

 glBindTexture(GL_TEXTURE_2D, texture[0]);   // choose the texture to use.

 glColor3f(1,1,1);

 glBegin(GL_TRIANGLES);

// cv::Point patch_center(patch_center_id%cloud.width,patch_center_id/cloud.width);
// ROS_INFO("center: %i %i", patch_center.x , patch_center.y);

 for (uint i=0; i<triangles.size(); i++){

  pcl_Point pts[3];
  float us[3];
  float vs[3];

  bool all_inside = true;

  for (uint j = 0; j<3; ++j){

   int idx = triangles[i][j];


   int x = idx%cloud.width;
   int y = idx/cloud.width;

   cv::Vec2f uv = uv_values.at<cv::Vec2f>(y,x);

   us[j] = ((uv.val[0]*uv_inv_scale)+1)/2;
   vs[j] = ((uv.val[1]*uv_inv_scale)+1)/2;

   // only draw triangle if all points are within the [0,1]^2-area
   if (us[j] < 0 || 1 < us[j] || vs[j] < 0 || 1 < vs[j]){
    all_inside = false;
    break;
   }

   pts[j] = cloud.points[idx];
  }

  if (all_inside){

   for (uint j=0; j<3; ++j){

//    float diff = norm(sub(pts[j],pts[(j+1)%3]));
//    if (diff>0.02){
//     ROS_INFO("Large edge (i=%i): %f %f %f, %f %f %f",i,pts[j].x, pts[j].y, pts[j].z,pts[(j+1)%3].x, pts[(j+1)%3].y, pts[(j+1)%3].z );
//    }

    glTexCoord2f(us[j],vs[j]);
    glVertex3f(pts[j].x, pts[j].y, pts[j].z);
   }
  }


 }

 glEnd();

 glDisable(GL_TEXTURE_2D);

}


void GL_Mesh_Viewer::drawMeshWithTexture(){

 glEnable(GL_TEXTURE_2D);

 if (texture_cv.cols != 256)
  LoadGLTextures();


 Cloud cloud;
 pcl::fromROSMsg(mesh->cloud, cloud);

 glBindTexture(GL_TEXTURE_2D, texture[0]);   // choose the texture to use.

 //ROS_INFO("w.h: %i %i", cloud.width, cloud.height);


 glBegin(GL_TRIANGLES);

 for (uint i=0; i<mesh->polygons.size(); i++){
  for (uint j = 0; j<3; ++j){

   int idx = mesh->polygons[i].vertices[j];

   pcl_Point p = cloud.points[idx];

   int x = idx%cloud.width;
   int y = idx/cloud.width;

   // ROS_INFO("x,y: %i %i u,v: %f %f", x,y,x/(1.0*(cloud.width-1)), y/(1.0*(cloud.height-1)));

   glTexCoord2f(x/(1.0*(cloud.width-1)),y/(1.0*(cloud.height-1)));
   glVertex3f(p.x, p.y, p.z);
  }
 }

 glEnd();


 glDisable(GL_TEXTURE_2D);

 // // Front Face (note that the texture's corners have to match the quad's corners)
 //
 // float x = 0.1;
 // float y = 0.1;
 //
 // // glColor3f(1,0,0);
 //
 // glTexCoord2f(0.0f, 0.0f);
 // glVertex3f(-x, -y,  0.0f);  // Bottom Left Of The Texture and Quad
 //
 // glTexCoord2f(0.0f, 1.0f);
 // glVertex3f( x, -y,  0.0f);  // Bottom Right Of The Texture and Quad
 //
 // glTexCoord2f(1.0f, 1.0f);
 // glVertex3f( x,  y,  0.0f);  // Top Right Of The Texture and Quad
 //
 // glTexCoord2f(1.0f, 0.0f);
 // glVertex3f(-x,  y,  0.0f);  // Top Left Of The Texture and Quad



}


/// Draw mesh with Triangle_Strip
/**
* @todo check for holes in the mesh
*/
void GL_Mesh_Viewer::drawMeshStrip(){

 Cloud cloud;
 pcl::fromROSMsg(mesh->cloud, cloud);

 pcl_Point p;

 for (uint x = 0; x<cloud.width-1; ++x){

  glBegin(GL_TRIANGLE_STRIP);

  for (uint y = 0; y<cloud.height; ++y){
   p = cloud.at(x,y);
   glColor3f(p.r/255.0, p.g/255.0, p.b/255.0);
   glVertex3f(p.x, p.y, p.z);


   p = cloud.at(x+1,y);
   glColor3f(p.r/255.0, p.g/255.0, p.b/255.0);
   glVertex3f(p.x, p.y, p.z);
  }

  glEnd();

 }

}



void GL_Mesh_Viewer::drawMesh(){


 // 0.06 ms
 Cloud cloud;
 pcl::fromROSMsg(mesh->cloud, cloud);


 bool normals_valid = (normals.size() == cloud.size());

 glBegin(GL_TRIANGLES);


 std::vector<pcl::Vertices>::iterator it = mesh->polygons.begin();
 std::vector<pcl::Vertices>::iterator end = mesh->polygons.end();
 pcl_Point* p;
 pcl_Normal* n;

 int undrawn_triangle_cnt = 0;

 for (;it != end; ++it){


  // TODO: compute normals for all points
  // HACK: use 1,0,0 for points without normals
  if (normals_valid){
   bool nan = false;
   for (uint i=0; i<3; ++i){
    n = &normals.points[it->vertices[i]];
    if  (n->normal_x != n->normal_x){
     undrawn_triangle_cnt++;
     nan = true;
     break;
    }
   }
   if (nan) continue;
  }

  for (uint i=0; i<3; ++i){
   p = &cloud.points[it->vertices[i]];
   glColor3f(p->r/255.0, p->g/255.0, p->b/255.0);

   if (normals_valid){
    n = &normals.points[it->vertices[i]];
    glNormal3f(n->normal_x, n->normal_y, n->normal_z);

    assert(n->normal_x == n->normal_x);

    //     ROS_INFO("pos: %f %f %f, normal: %f %f %f",p->x, p->y, p->z,n->normal_x, n->normal_y, n->normal_z);
   }

   glVertex3f(p->x, p->y, p->z);
  }

  //  p = &cloud.points[it->vertices[1]];
  //  glColor3f(p->r/255.0, p->g/255.0, p->b/255.0);
  //  glVertex3f(p->x, p->y, p->z);
  //
  //  p = &cloud.points[it->vertices[2]];
  //  glColor3f(p->r/255.0, p->g/255.0, p->b/255.0);
  //  glVertex3f(p->x, p->y, p->z);
 }

 glEnd();


 // ROS_INFO("Couldn't draw %i of %zu triangles because of missing normals", undrawn_triangle_cnt,mesh->polygons.size());


}



void GL_Mesh_Viewer::drawList(GLuint list_id){
 glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 glEnable( GL_DEPTH_TEST );
 // glLoadIdentity();
 // glTranslatef(0.0, 0.0, -10.0);
 // // set the zoom according to the scale variable
 // glScalef(scale, scale, scale);
 // // set the image rotation up according to xRot, yRot, zRot
 // glRotatef( xRot, 1.0, 0.0, 0.0);
 // glRotatef( yRot, 0.0, 1.0, 0.0);
 // glRotatef( zRot, 0.0, 0.0, 1.0);

 glCallList(list_id);
}



void glVectorToCvMat(GLdouble* v, cv::Mat &M){
 for (uint i=0; i<16; ++i){
  M.at<double>(i%4,i/4) = v[i];
 }
}


cv::Point2f GL_Mesh_Viewer::simulateGlPipeline(float x, float y, float z){


 GLdouble v[16];

 cv::Mat MV(4,4,CV_64FC1);
 glGetDoublev(GL_MODELVIEW_MATRIX,v);


 ROS_INFO("With openGL 2.0");
 for (uint x_=0; x_<4; ++x_){
  for (uint y=0; y<4; ++y)
   cout << v[y*4+x_] << " ";
  cout << endl;
 }

 glVectorToCvMat(v,MV);

 cv::Mat P(4,4,CV_64FC1);
 glGetDoublev(GL_PROJECTION_MATRIX,v);
 glVectorToCvMat(v,P);

 cout << "GL_PROJECTION " << endl << P << endl;


 // ROS_INFO("GL_PROJECTION");
 // for (uint x_=0; x_<4; ++x_){
 //  for (uint y=0; y<4; ++y)
 //   cout << v[y*4+x_] << " ";
 //  cout << endl;
 // }


 cv::Mat Pos(4,1,CV_64FC1);
 Pos.at<double>(0) = x;
 Pos.at<double>(1) = y;
 Pos.at<double>(2) = z;
 Pos.at<double>(3) = 1;

 cout << "MV: " << MV << endl;

 cv::Mat local = MV*Pos;

 ROS_INFO("simulating for %f %f %f", x,y,z);
 cout << "local frame: " << local << endl;

 cv::Mat dc = P*local;

 cout << "dc: " << dc << endl;


 dc /= dc.at<double>(3);


 cout << "ndc: " << dc << endl;

 cv::Point2f res;

 res.x = (dc.at<double>(0)+1)/2*w_;
 res.y = (dc.at<double>(1)+1)/2*h_;

 cout << "pixel: " << res.x << "  " << res.y << endl;

 cv::Mat cor = proj_matrix*Pos;

 cor /= cor.at<double>(2);

 ROS_INFO("original: %f %f %f", cor.at<double>(0),cor.at<double>(1),cor.at<double>(2));


 return res;
}



void GL_Mesh_Viewer::setUpIlumination(){

 // glEnable(GL_COLOR_MATERIAL);
 glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

 GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
 GLfloat mat_shininess[] = { 50.0 };
 glClearColor (0.0, 0.0, 0.0, 0.0);
 glShadeModel (GL_SMOOTH);

 glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
 glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);



 // light_z_pos += 0.01;

 // GLfloat light_position_1[] = { 0.0, 0.0, 2, 1.0 };

 // GLfloat dir_1[] = { 0.0, 0.0, light_z_pos>0?1:-1};


 GLfloat color_red[]   = { 1.0, 0.0,  0.0, 1.0 };
 GLfloat color_blue[]  = { 0.0, 0.0,  1.0, 1.0 };
 // GLfloat color_green[] = { 0.0, 1.0,  0.0, 1.0 };
 // GLfloat color_black[] = { 0.0, 0.0,  0.0, 0.0 };
 // GLfloat white[] = { 1.0, 1.0,  1.0, 1.0 };


 // GLfloat color_red[] = { 50.0 };
 glMatrixMode(GL_MODELVIEW_MATRIX);
 glPushMatrix();
 glLoadIdentity();
 //
 // // glTranslatef(0,0,light_z_pos);
 //
 // gluLookAt(0,0,light_z_pos, 0,0,0,0,1,0);

 ROS_INFO("XXXXXXXXXXXXXXXXX  light pos: %f", light_z_pos);


 // glLightfv(GL_LIGHT1, GL_POSITION, light_position_1); // position
 // glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse_color_green); // color of diffuse light
 // glLightfv(GL_LIGHT1, GL_SPECULAR, diffuse_color_green); // color of specular light
 // glLightfv(GL_LIGHT1, GL_AMBIENT, diffuse_color_blue); // color of ambient light


 // glLightfv(GL_LIGHT2, GL_POSITION, light_position_2); // position
 // glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse_color_red); // color of diffuse light
 // glLightfv(GL_LIGHT2, GL_SPECULAR, diffuse_color_green); // color of specular light
 // glLightfv(GL_LIGHT2, GL_AMBIENT, diffuse_color_red); // color of specular light



 // glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, dir_1);
 //
 // GLfloat angle = 5;
 // glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, angle);

 // glLightfv(GL_LIGHT2, GL_POSITION, light_position_2);
 // glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse_color_blue);


 // float x,y,z;
 // x = 0;
 // y = light_z_pos;
 //
 // light_z_pos += 0.01;
 //
 // z = 0.5;


 glTranslatef(0,0,1);
 GLfloat light_position_2[] = { 0, 0, -1, 0.0 };
 glLightfv(GL_LIGHT0, GL_POSITION, light_position_2);
 glLightfv(GL_LIGHT0, GL_AMBIENT,  color_blue);
 glLightfv(GL_LIGHT0, GL_DIFFUSE,  color_red);

 // glLightfv(GL_LIGHT0, GL_SPECULAR, color_green);
 // glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light_position_2);
 // GLfloat angle = 30;
 // glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, angle);

 glPopMatrix();

 glEnable(GL_LIGHTING);
 glEnable(GL_LIGHT0);
 // glEnable(GL_LIGHT1);
 // glEnable(GL_LIGHT2);
 glEnable(GL_DEPTH_TEST);


}


void GL_Mesh_Viewer::setUpMapImage(){

 glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 glEnable( GL_DEPTH_TEST );

 glMatrixMode(GL_MODELVIEW_MATRIX);
 glLoadIdentity();

 glMatrixMode(GL_PROJECTION);
 glLoadIdentity();

 glOrtho(grid_min_x,grid_min_x+grid_width,grid_min_y+grid_height,grid_min_y,-1,1);
 glViewport(0,0,img_width,img_height);

}


void GL_Mesh_Viewer::setUpProjectorImage(){

 glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 glEnable( GL_DEPTH_TEST );

 GLdouble model_view[16];


 for (uint x=0; x<3; ++x){
  for (uint y=0; y<4; ++y){

   if (x < 2)
    model_view[y*4+x] = proj_matrix.at<double>(x,y);
   else{
    model_view[y*4+(x+1)] = proj_matrix.at<double>(x,y);
    model_view[y*4+x] = 0;
   }
  }
 }

 model_view[10] = 1; // MV(3,3) = 1


 glMatrixMode(GL_MODELVIEW_MATRIX);
 glLoadIdentity();
 glMultMatrixd(model_view);


 glMatrixMode(GL_PROJECTION);
 glLoadIdentity();
 glOrtho(0,w_,h_,0,-10,10);

 glViewport(0,0,w_,h_);


}



void GL_Mesh_Viewer::paintGL()
{

 // setUpIlumination();

 // ros::Time now_render = ros::Time::now();
 // if (draw_map)
 //  setUpMapImage();
 // else
 setUpProjectorImage();


 // drawing path of ant as GlLine
 // drawPath();

 // drawAnts();

 // drawing rest of scene (texture or glTriangles with height dependent color)
 // drawMesh();

 if (show_texture){
  // ROS_INFO("showing texture");
  showExpMapTexture();
 }else
  drawMeshStrip();



 // if (height_lines.size() > 0){
 //  drawHeightLines();
 // }
 //
 //
 // if (path.size() > 0)
 //  drawPath();
 //  if (show_texture)
 // else {
 //   drawMeshWithTexture();
 //  else
 //   drawMesh();
 // }


}



GLuint GL_Mesh_Viewer::makeObject()
{



 GLuint list;
 list = glGenLists(1);
 glNewList(list, GL_COMPILE);
 // set the initial color
 glColor3f( 1.0, 0.0, 0.0 );
 // points for triangle to draw Sierpinski Gasket
 //Point a, b, c;
 //a.x=-0.5; a.y=-0.5;
 //b.x=0.5; b.y=-0.5;
 //c.x=0.0; c.y=0.5;
 // drawSierpinski(a,b,c,3);


 float l = 0.2;
 float z = 0;

 glBegin(GL_POLYGON);
 glVertex3f(l,l,z);
 glVertex3f(l,-l,z);
 glVertex3f(-l,-l,z);
 glVertex3f(-l,l,z);
 glEnd();


 //l = 0.2;
 //z = -1;
 //
 //glBegin(GL_POLYGON);
 //glVertex3f(l,l,z);
 //glVertex3f(l,-l,z);
 //glVertex3f(-l,-l,z);
 //glVertex3f(-l,l,z);
 //glEnd();



 glColor3f( 0.0, 1.0, 0.0 );

 l = 0.05;
 z = 0.05;

 glBegin(GL_POLYGON);
 glVertex3f(l,l,z);
 glVertex3f(l,-l,z);
 glVertex3f(-l,-l,z);
 glVertex3f(-l,l,z);
 glEnd();

 //l = 0.3;
 //z = 0.1;
 //glColor3f( 1.0, 1.0, 0.0 );
 //glBegin(GL_POLYGON);
 //glVertex3f(l,l,z);
 //glVertex3f(l,-l,z);
 //glVertex3f(-l,-l,z);
 //glVertex3f(-l,l,z);
 //glEnd();


 float dx = 0.2;

 glColor3f( 0.0, 0.0, 1.0 );
 glBegin(GL_POLYGON);
 glVertex3f(l+dx,l,z);
 glVertex3f(l+dx,-l,z);
 glVertex3f(-l+dx,-l,z);
 glVertex3f(-l+dx,l,z);
 glEnd();

 float dy = 0.2;
 dx = 0;


 glColor3f( 0.0, 1.0, 1.0 );
 glBegin(GL_POLYGON);
 glVertex3f(l+dx,l+dy,z);
 glVertex3f(l+dx,-l+dy,z);
 glVertex3f(-l+dx,-l+dy,z);
 glVertex3f(-l+dx,l+dy,z);
 glEnd();




 glEndList();
 return list;
} // end make object


/*--------------------------------------------------------------------------*/
/**
* Set up the OpenGL rendering state, and define display list
*/

void GL_Mesh_Viewer::initializeGL()
{
 glClearColor( 0.0, 0.0, 0.0, 0.0 ); // Let OpenGL clear to black
 object = makeObject(); // generate an OpenGL display list
 glShadeModel( GL_SMOOTH ); // we want smooth shading . . . try GL_FLAT if you like
}

/*--------------------------------------------------------------------------*/
/**
* Set up the OpenGL view port, matrix mode, etc.
*/
void GL_Mesh_Viewer::resizeGL( int w, int h )
{
 w_ = w;
 h_ = h;

 glViewport( 0, 0, (GLint)w, (GLint)h );
 glMatrixMode( GL_PROJECTION );
 glLoadIdentity();
 glFrustum(-1.0,1.0,-2.0,2.0, 1, 3);
 glMatrixMode( GL_MODELVIEW );
}



