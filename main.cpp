#include <stdio.h>
#include <string>
#include <iostream>
#include <GL/freeglut.h>
#include <GL/glu.h>
#include "gl_util.h"
#include "material.h"
#include "delta_gl.h"
#include "svector.h"
#include "delta.h"
#include <cmath>

const size_t WindowsWidth = 1024;
const size_t WindowsHeight = 1024;

GLfloat lightPosition0[] = {3000.0, 1000.0, 800.0, 0.0};
GLfloat lightPosition1[] = {-3000.0, -1000.0, 1000.0, 0.0};
GLfloat whiteLight[] = {1.0, 1.0, 1.0, 1.0};
GLfloat lModelAmbient[] = {0.9, 0.9, 0.9, 1.0};


DeltaPrinter printer;
DeltaPrinterGL deltaGL(printer);
int errMat = 0;


class ViewPort {
public:
  ViewPort(float x, float y, float w, float h, float fov)
    : x(x), y(y), w(w), h(h), fov(fov){
    qCamera(0, 0, 0, 1);
    mouse_vx = 0;
    mouse_vy = 0;
    mouse_vz = -1000;
    //svector::float4 qNew(0, 0, 0, 1);
    //qNew.euler(3.1415 / 2, 0, 0);
    //qCamera.quaternion_mult(qNew);
  }



  void render() {
    if (0) {
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      glViewport(x, y, w, h);
      gluPerspective(fov, w / h, 1, 2000);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();

      glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);
      glLightfv(GL_LIGHT1, GL_POSITION, lightPosition1);

      glTranslatef(mouse_vx, -mouse_vy, mouse_vz);
      float pi = 3.14159265358;
      svector::float4 qNew(0, 0, 0, 1);
      qNew.euler(mouse_dy / 2/h, mouse_dx /2/ w, 0.0);
      qCamera.quaternion_mult(qNew);
      svector::float4 qAxis = qCamera.axis();

      glRotatef(qAxis.w / pi * 180, qAxis.x, qAxis.y, qAxis.z);
      glTranslatef(0,0,-400);

      glDisable(GL_LIGHTING);
      draw_axes_positive(500, 500, 500);
      glEnable(GL_LIGHTING);
      
      // deltaGL.render();

      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    glDisable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(x, y, w, h);
    glOrtho(0, w, h, 0, -10, 10);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPushMatrix();
    float dx = (WindowsWidth - (WindowsHeight - 100) ) / 2.0f;
    glTranslatef(dx, 0, 0);
    deltaGL.renderTexture(WindowsHeight - 100, WindowsHeight - 100, errMat);
    glPopMatrix();
  }

  void mouse_button(int button, int status, int xm, int ym) {
    if (xm < x || xm > x + w || ym < y || ym > y + h) return;
    left_button_status = GLUT_UP;
    right_button_status = GLUT_UP;
    if ((button == 3) || (button == 4)) {
      if (status == GLUT_DOWN) {
        mouse_vz += button == 3 ? 19 : -19;
        if (mouse_vz > 40) mouse_vz = 30;
        if (mouse_vz < -2000) mouse_vz = -2000;
      }
    }
    else {
      if (button == GLUT_LEFT_BUTTON) {
        if (status == GLUT_DOWN) {
          left_button_status = GLUT_DOWN;
          left_button_down_x = xm;
          left_button_down_y = ym;
        }
      }
      if (button == GLUT_RIGHT_BUTTON) {
        if (status == GLUT_DOWN) {
          right_button_status = GLUT_DOWN;
          left_button_down_x = xm;
          left_button_down_y = ym;
        }
      }
    }
  }

  void mouse_active_motion(int xm, int ym) {
    if (xm < x || xm > x + w || ym < y || ym > y + h) return;
    if (left_button_status == GLUT_DOWN) {
      mouse_dx = (xm - left_button_down_x);
      mouse_dy = -(ym - left_button_down_y);
      if (fabs(mouse_dx) < 2) mouse_dx = 0;
      if (fabs(mouse_dy) < 2) mouse_dy = 0;
    }
    if (right_button_status == GLUT_DOWN) {
      float dx = (xm - left_button_down_x) / float(1);
      float dy = -(ym - left_button_down_y) / float(1);
      mouse_vy += dy;
      mouse_vx += dx;
    }

    left_button_down_x = xm;
    left_button_down_y = ym;
  }

  void mouse_passive_motion(int xm, int ym) {
    if (xm < x || xm > x + w || ym < y || ym > y + h) return;
    left_button_down_y = ym;
    left_button_down_x = xm;
  }
private:
  float x, y, w, h;
  float fov;

  svector::float4 qCamera;
  int left_button_status;
  int right_button_status;
  int left_button_down_x;
  int left_button_down_y;
  float mouse_dx;
  float mouse_dy;
  float mouse_vx;
  float mouse_vy;
  float mouse_vz;
};

ViewPort view(0, 0, WindowsWidth, WindowsHeight, 60);

void display() {
  glClearColor(0, 0, 0, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  view.render();
  glutSwapBuffers();
}

void init_display() {
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_POLYGON_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

  glLightfv(GL_LIGHT0, GL_SPECULAR, whiteLight);        // sets specular light to white
  glLightfv(GL_LIGHT0, GL_DIFFUSE, whiteLight);         // sets diffuse light to white
  glLightfv(GL_LIGHT1, GL_SPECULAR, whiteLight);        // sets specular light to white
  glLightfv(GL_LIGHT1, GL_DIFFUSE, whiteLight);         // sets diffuse light to white
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lModelAmbient);    // global ambient light 
  
  glEnable(GL_LIGHTING);                    // enables lighting
  glEnable(GL_LIGHT0);                    // enables light0
  glEnable(GL_LIGHT1);                    // enables light0
  glShadeModel(GL_SMOOTH);
  // glEnable(GL_COLOR_MATERIAL);                // enables opengl to use glColor3f to define material color
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

void reshape(int w, int h) {
  init_display();
  display();
}

void mouse_wheel(int wheel, int direction, int x, int y) {
  int y1 = WindowsHeight - y;
  view.mouse_button(direction > 0 ? 3 : 4, GLUT_DOWN, x, y1);
}

void mouse_button(int button, int state, int x, int y) {
  int y1 = WindowsHeight - y;
  view.mouse_button(button, state, x, y1);
}

void mouse_active_motion(int x, int y) { 
  int y1 = WindowsHeight - y;
  view.mouse_active_motion(x, y1); 
}

void mouse_passive_motion(int x, int y) {
  int y1 = WindowsHeight - y;
  view.mouse_passive_motion(x, y1); 
}

void normal_keys(unsigned char key, int x, int y) {
  switch (key) {
  case 'm': {
    printer.surfaceMap();
      // DeltaGeometry geo;
      // float za, zb, zc;
      // float x, y, z;
      // printer.inverseKinematics(10, 10, 10, za, zb, zc, geo);
      // printf("IK(%f, %f, %f)\n", za, zb, zc);
      // printer.forwardKinematics(za, zb, zc, x, y, z, geo);
      // printf("FK(%f, %f, %f)\n", x, y, z);
    }
    break;    
  case '1':
    errMat = 0;
    break;
  case '2':
    errMat = 1;
    break;
  case '3':
    errMat = 2;
    break;
  case '4':
    errMat = 3;
    break;
  case '6':
    deltaGL.setRenderMode(DeltaPrinterGL::DETAILED); 
    break;
  case '7':
    deltaGL.setRenderMode(DeltaPrinterGL::SIMPLIFIED);
    break;
  case 'a':
    //printer.setCarriages(400, 400, 400);
    break;
  case 'z':
    //printer.moveCarriages(-10, 0, 0);
    break;    
  case 's':
    //printer.moveCarriages(0, 10, 0);
    break;
  case 'x':
    //printer.moveCarriages(0, -10, 0);
    break;    
  case 'd':
    //printer.moveCarriages(0, 0, 10);
    break;
  case 'c':
    //printer.moveCarriages(0, 0, -10);
    break;
  case 'f':
    //printer.moveCarriages(10, 10, 10);
    break;
  case 'v':
    //printer.moveCarriages(-10, -10, -10);
    break;
  case 't':
    //printer.setXYZ(0, 180, 0);
    break;    
  case 'y':
    //printer.setXYZ(0, 0, 0);
    break;    
  case 'u':
    //printer.setXYZ(0, 0, 30.492798);
    break;        
  case 27:
    glutLeaveMainLoop();
    break;
  default:
    break;
  }
}

void init_glut_window(int argc, char *argv[]) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
  //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(WindowsWidth, WindowsHeight);
  glutCreateWindow("Delta Sim");

  glutDisplayFunc(display);
  glutIdleFunc(display);
  glutKeyboardFunc(normal_keys);
  //glutSpecialFunc(special_keys);
  glutMouseFunc(mouse_button);
  glutMotionFunc(mouse_active_motion);
  glutPassiveMotionFunc(mouse_passive_motion);
  glutMouseWheelFunc(mouse_wheel);
  glutReshapeFunc(reshape);
  deltaGL.init();

  glutMainLoop();
}


int main(int argc, char *argv[]) {
  init_glut_window(argc, argv);
}
