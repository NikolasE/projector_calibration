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


//#include "projector_calibration/glfractal.h"

/**
 * Constructor that creates a GLFractal widget
 */
GLFractal::GLFractal( QWidget* parent)
: QGLWidget(parent)
{
 xRot = yRot = zRot = 0.0;
 scale = 1.25;
 object = 0;
}

GLFractal::~GLFractal()
{
 glDeleteLists( object, 1 );
}


/*
 * Create new Object list for meshLines
 *
 */
GLuint GLFractal::createMeshList(const visualization_msgs::Marker& mesh_marker){
 GLuint list = glGenLists(1);
 //assert(list != 0);
 glNewList(list, GL_COMPILE);
 object = list;

 assert(mesh_marker.points.size() % 3 == 0);


 // (fast) jede Kante wird zwei Mal gezeichnet..

 for (uint i=0; i<mesh_marker.points.size(); i+=3){

  glBegin(GL_LINE_LOOP);


  geometry_msgs::Point pt0 = mesh_marker.points[i];
  geometry_msgs::Point pt1 = mesh_marker.points[i+1];
  geometry_msgs::Point pt2 = mesh_marker.points[i+2];

  std_msgs::ColorRGBA c0 = mesh_marker.colors[i];
  std_msgs::ColorRGBA c1 = mesh_marker.colors[i+1];
  std_msgs::ColorRGBA c2 = mesh_marker.colors[i+2];

  assert(pt0.x == pt0.x);
  assert(pt1.x == pt1.x);
  assert(pt2.x == pt2.x);


  glVertex3f(pt0.x, pt0.y, pt0.z);
  glColor3f(c0.r, c0.g, c0.b);

  glVertex3f(pt1.x, pt1.y, pt1.z);
  glColor3f(c1.r, c1.g, c1.b);


  glVertex3f(pt2.x, pt2.y, pt2.z);
  glColor3f(c2.r, c2.g, c2.b);


  glEnd();


 }


 glEndList();
 return list;
}

void GLFractal::drawList(GLuint list_id){
 glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 glEnable( GL_DEPTH_TEST );
 glLoadIdentity();
 glTranslatef(0.0, 0.0, -10.0);
 // set the zoom according to the scale variable
 glScalef(scale, scale, scale);
 // set the image rotation up according to xRot, yRot, zRot
 glRotatef( xRot, 1.0, 0.0, 0.0);
 glRotatef( yRot, 0.0, 1.0, 0.0);
 glRotatef( zRot, 0.0, 0.0, 1.0);

 glCallList(list_id);
}




/*--------------------------------------------------------------------------*/
/**
 * Paint the box. The actual openGL commands for drawing the box are
 * performed here.
 */
void GLFractal::paintGL()
{
 glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 glEnable( GL_DEPTH_TEST );
 glLoadIdentity();
 glTranslatef(0.0, 0.0, -8.0);
 // set the zoom according to the scale variable
 glScalef(scale, scale, scale);
 // set the image rotation up according to xRot, yRot, zRot
 glRotatef( xRot, 1.0, 0.0, 0.0);
 glRotatef( yRot, 0.0, 1.0, 0.0);
 glRotatef( zRot, 0.0, 0.0, 1.0);

 glCallList(object);
}

GLuint GLFractal::makeObject()
{  GLuint list;
list = glGenLists(1);
glNewList(list, GL_COMPILE);
// set the initial color
glColor3f( 1.0, 0.0, 0.0 );
// points for triangle to draw Sierpinski Gasket
Point a, b, c;
a.x=-0.5; a.y=-0.5;
b.x=0.5; b.y=-0.5;
c.x=0.0; c.y=0.5;
drawSierpinski(a,b,c,3);
glEndList();
return list;
} // end make object


/*--------------------------------------------------------------------------*/
/**
 * Set up the OpenGL rendering state, and define display list
 */

void GLFractal::initializeGL()
{
 glClearColor( 0.0, 0.0, 0.0, 0.0 ); // Let OpenGL clear to black
 //  object = makeObject(); // generate an OpenGL display list
 glShadeModel( GL_SMOOTH ); // we want smooth shading . . . try GL_FLAT if you like
}

/*--------------------------------------------------------------------------*/
/**
 * Set up the OpenGL view port, matrix mode, etc.
 */
void GLFractal::resizeGL( int w, int h )
{
 glViewport( 0, 0, (GLint)w, (GLint)h );
 glMatrixMode( GL_PROJECTION );
 glLoadIdentity();
 glFrustum(-1.0,1.0,-1.0,1.0,5.0,15.0);
 glMatrixMode( GL_MODELVIEW );
}

//*********************************************************
// Functions to rotate the model along the x, y, & z axes.
void GLFractal::setXRotation(int degrees)
{  xRot=(GLfloat)(degrees % 360);
updateGL();
}

void GLFractal::setYRotation(int degrees)
{  yRot=(GLfloat)(degrees % 360);
updateGL();
}

void GLFractal::setZRotation(int degrees)
{  zRot=(GLfloat)(degrees % 360);
updateGL();
}

//**********************************************************
// Write Your Own Function To Set The Scale Here!!!!
//void GLFractal::setScale(int newscale){ }

//**********************************************************
// Draws a triangle
void GLFractal::drawTriangle(Point a, Point b, Point c)
{  glBegin(GL_POLYGON);
glVertex2f(a.x,a.y);
glVertex2f(b.x,b.y);
glVertex2f(c.x,c.y);
glEnd();
} // end draw triangle

//*******************************************************
// Function from class to draw the Sierpinski fractal
//   Recursion is FUN!
void GLFractal::drawSierpinski(Point a, Point b, Point c, int level)
{  Point m0, m1, m2;

if (level > 0) {
 m0.x = (a.x+b.x) /2.0;
 m0.y = (a.y+b.y) /2.0;
 m1.x = (a.x+c.x) /2.0;
 m1.y = (a.y+c.y) /2.0;
 m2.x = (b.x+c.x) /2.0;
 m2.y = (c.y+b.y) /2.0;
 drawSierpinski(a,m0,m1,level-1);
 drawSierpinski(b,m2,m0,level-1);
 drawSierpinski(c,m1,m2,level-1);
} else drawTriangle(a,b,c);
} // end draw Sierpinski

