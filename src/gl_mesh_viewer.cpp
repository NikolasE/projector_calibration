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

 glBindTexture(GL_TEXTURE_2D, texture[0]);   // 2d texture (x and y size)

 glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
 glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); // scale linearly when image smalled than texture


 texture_cv = cv::imread("imgs/test.bmp");

 ROS_INFO("texture: %i %i", texture_cv.cols, texture_cv.rows);

 // 2d texture, level of detail 0 (normal), 3 components (red, green, blue), x size from image, y size from image,
 // border 0 (normal), rgb color data, unsigned byte data, and finally the data itself.
 glTexImage2D(GL_TEXTURE_2D, 0, 3, texture_cv.cols, texture_cv.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, texture_cv.data);

 // cv::namedWindow("foo");
 // cv::imshow("foo", texture_cv);
 // cv::waitKey(10);

};



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


void GL_Mesh_Viewer::drawMesh(){

 Cloud cloud;
 pcl::fromROSMsg(mesh->cloud, cloud);

 glBegin(GL_TRIANGLES);

 for (uint i=0; i<mesh->polygons.size(); i++){
  for (uint j = 0; j<3; ++j){
   pcl_Point p = cloud.points[mesh->polygons[i].vertices[j]];
   glColor3f(p.r/255.0, p.g/255.0, p.b/255.0);
   glVertex3f(p.x, p.y, p.z);
  }
 }

 glEnd();

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


 // ROS_INFO("GL_PROJECTIN");
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

 cv::Mat cor = M*Pos;

 cor /= cor.at<double>(2);

 ROS_INFO("original: %f %f %f", cor.at<double>(0),cor.at<double>(1),cor.at<double>(2));


 return res;
}



/*--------------------------------------------------------------------------*/
/**
 * Paint the box. The actual openGL commands for drawing the box are
 * performed here.
 */
void GL_Mesh_Viewer::paintGL()
{

 glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 glEnable( GL_DEPTH_TEST );

 GLdouble model_view[16];


 // cout << "P " << M << endl;

 for (uint x=0; x<3; ++x){
  for (uint y=0; y<4; ++y){

   if (x < 2)
    model_view[y*4+x] = M.at<double>(x,y);
   else{
    model_view[y*4+(x+1)] = M.at<double>(x,y);
    model_view[y*4+x] = 0;
   }
  }
 }

 model_view[10] = 1; // MV(3,3) = 1

 // cout << "Modelview" << endl;
 // for (uint x=0; x<4; ++x){
 //  for (uint y=0; y<4; ++y)
 //   cout << model_view[y*4+x] << " ";
 //  cout << endl;
 // }

 glMatrixMode(GL_MODELVIEW_MATRIX);
 glLoadIdentity();
 glMultMatrixd(model_view);

 // glGetDoublev(GL_MODELVIEW_MATRIX,model_view);
 // ROS_INFO("With openGL");
 // for (uint x=0; x<4; ++x){
 //  for (uint y=0; y<4; ++y)
 //   cout << model_view[y*4+x] << " ";
 //  cout << endl;
 // }

 glMatrixMode(GL_PROJECTION);
 glLoadIdentity();
 glOrtho(0,w_,h_,0,-10,10);
 glViewport(0,0,w_,h_);

 //

 if (show_texture)
  drawMeshWithTexture();
 else
  drawMesh();

 //glCallList(object);


}


//void GL_Mesh_Viewer::setMesh(pcl::PolygonMesh* mesh_){
// mesh = mesh_;
//}


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



