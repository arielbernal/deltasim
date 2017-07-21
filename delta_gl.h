#ifndef _DELTA_GL_H
#define _DELTA_GL_H

#include <cstring>
#include "delta.h"
#include "material.h"
#include "gl_util.h"

class DeltaPrinterGL {
public:
  enum RenderMode {
    SIMPLIFIED,
    DETAILED
  };

  DeltaPrinterGL(DeltaPrinter& delta) : delta(delta), renderMode(0) {
    texDat = new int8_t[delta.sm_height * delta.sm_width * 3];
    std::memset(texDat, 0, delta.sm_height * delta.sm_width * 3);
  }
  ~DeltaPrinterGL() {
    delete[] texDat;
  }

  void init() {
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, delta.sm_width, delta.sm_height, 0, GL_RGB, GL_UNSIGNED_BYTE, texDat);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
  }

  void setRenderMode(int i) {
    renderMode = i;
  }
/*
  void render() {
    drawBed();
    drawFrame();
    if (renderMode == SIMPLIFIED) {
      drawEffectorPoint();
      drawEquivalentRods();
    }
    else {
      drawRods();
      drawEffector();
      drawNozzle();
    }
  }
  */
  void colorNode(float ColorVal, float ColorMin, float ColorMax, float& r, float& g, float& b) {
    if (ColorVal<ColorMin) {r = 0; g = 0; b = 1; }
    if (ColorVal>ColorMax) {r = 1; g = 0; b = 0; }

    if ( (ColorVal>=3*ColorMin/3+0*ColorMax/3) && (ColorVal<=2*ColorMin/3+1*ColorMax/3) ) {
      r = 0; g = 3*(ColorVal-ColorMin)/(ColorMax-ColorMin); b = 1;
    }
    if ( (ColorVal>=2*ColorMin/3+1*ColorMax/3) && (ColorVal<=1*ColorMin/2+1*ColorMax/2) ) {
      r = 0; g = 1; b = (3*ColorMax+3*ColorMin-6*ColorVal)/(ColorMax-ColorMin);
    }
    if ( (ColorVal>=1*ColorMin/2+1*ColorMax/2) && (ColorVal<=1*ColorMin/3+2*ColorMax/3) ) {
      r = (6*ColorVal-3*ColorMax-3*ColorMin)/(ColorMax-ColorMin); g = 1; b = 0;
    }
    if ( (ColorVal>=1*ColorMin/3+2*ColorMax/3) && (ColorVal<=0*ColorMin/3+3*ColorMax/3) ) {
      r = 1; g = 3*(ColorMax-ColorVal)/(ColorMax-ColorMin); b = 0;
    }
  }


  void renderTexture(int w, int h, int errMat) {
    float fmax = -10E10;
    float fmin = 10E10;
    float* arr;
    char *sErrMat;
    if (errMat == 0) { arr = delta.smx; sErrMat = "x-Error"; }
    if (errMat == 1) { arr = delta.smy; sErrMat = "y-Error"; }
    if (errMat == 2) { arr = delta.smz; sErrMat = "z-Error"; }
    if (errMat == 3) { arr = delta.smxy; sErrMat = "xy-Error"; }

    for (int i = 0; i < delta.sm_height; ++i) {
      for (int j = 0; j < delta.sm_width; ++j) {
        int idx = i * delta.sm_width + j;
        if (arr[idx] != 30000) {
          if (arr[idx] > fmax) fmax = arr[idx];
          else if (arr[idx] < fmin) fmin = arr[idx];
        }
      }
    }
    
    char text[2000];
    sprintf(text, "%s | Min = %8.3f mm | Max = %8.3f mm | Diff = %8.3f um\n", sErrMat, fmin, fmax, fabs(fmax-fmin) * 1000);
    renderString(10, h + 20, text);

    for (int i = 0; i < delta.sm_height; ++i) {
      for (int j = 0; j < delta.sm_width; ++j) {
        int idx = i * delta.sm_width + j;
        float r, g, b;
        if(arr[idx] != 30000) 
          colorNode(arr[idx], fmin, fmax, r, g, b);
        else
        {
          r = 0; g = 0; b = 0;
        }
        texDat[3 * idx + 0] = r * 255;
        texDat[3 * idx + 1] = g * 255;
        texDat[3 * idx + 2] = b * 255;
      }
    }
    //printf("fmin = %f fmax = %f\n", fmin, fmax);

    glBindTexture(GL_TEXTURE_2D, tex);
    glEnable(GL_TEXTURE_2D);
    glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, delta.sm_width, delta.sm_height, 0, GL_RGB, GL_UNSIGNED_BYTE, texDat);
    glBegin(GL_QUADS);
      glNormal3d(1, 0, 0);
      glTexCoord2i(0, 0); glVertex2i(0, 0);
      glTexCoord2i(1, 0); glVertex2i(w, 0);
      glTexCoord2i(1, 1); glVertex2i(w, h);
      glTexCoord2i(0, 1); glVertex2i(0, h);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  /*

  void drawBed() {
    chrome.active();
    glPushMatrix();
    glTranslatef(0, 0, -delta.BedHeight);
    drawCylinder(delta.BedHeight, delta.DeltaRadius);
    glPopMatrix();    
  }

  void drawFrame() {
    ruby.active();
    float angle[] = {delta.Alpha, delta.Beta, delta.Gamma};
    for (int i = 0; i < 3; ++i) {
      glPushMatrix();
        glRotatef(angle[i], 0, 0, 1);
        glTranslatef(0, 0, -delta.FrameBottomH - delta.BedHeight);
        glPushMatrix();
          glTranslatef(delta.TowerRadius + delta.FrameVerticalOffset, 0, delta.TowerHeight / 2);
          drawCube(delta.FrameVerticalH, delta.FrameVerticalW, delta.TowerHeight);
        glPopMatrix();
        glPushMatrix();
          glTranslatef(delta.TowerRadius / 2, 0, delta.FrameBottomH / 2);
          drawCube(delta.TowerRadius, delta.FrameBottomW, delta.FrameBottomH);
        glPopMatrix();
        glPushMatrix();
          glTranslatef(delta.TowerRadius / 2, 0, delta.TowerHeight - delta.FrameTopH / 2);
          drawCube(delta.TowerRadius, delta.FrameTopW, delta.FrameTopH);
        glPopMatrix();    
      glPopMatrix();        
    }
  }

  void drawEquivalentRods() {
    float XT[] = {delta.Xa, delta.Xb, delta.Xc};
    float YT[] = {delta.Ya, delta.Yb, delta.Yc};
    float ZT[] = {delta.Za, delta.Zb, delta.Zc};
    for (int i = 0; i < 3; ++i) {
      turquoise.active();  
      glBegin(GL_LINES);
        glVertex3f(delta.x, delta.y, delta.z);
        glVertex3f(XT[i], YT[i], ZT[i]);
      glEnd();
      float ddx = delta.x - XT[i];
      float ddy = delta.y - YT[i];
      float ddz = delta.z - ZT[i];
      float l = sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
      //printf("l[%d] = %f]\n", i, l);
      // draw joints
      gold.active();
      glPushMatrix();
        glTranslatef(delta.x, delta.y, delta.z);
        drawSphere(5, 10, 10);
      glPopMatrix();
      glPushMatrix();
        glTranslatef(XT[i], YT[i], ZT[i]);
        drawSphere(5, 10, 10);
      glPopMatrix();  
    }
  }

  void drawEffectorPoint() {
    gold.active();
    glPushMatrix();
      glTranslatef(delta.x, delta.y, delta.z);
      drawSphere(10, 10, 10);
    glPopMatrix();
  }

  void drawRods() {
    float angle[] = {delta.rAlpha, delta.rBeta, delta.rGamma};
    float sinAngle[] = {sin(delta.rAlpha), sin(delta.rBeta), sin(delta.rGamma)};
    float cosAngle[] = {cos(delta.rAlpha), cos(delta.rBeta), cos(delta.rGamma)};
    float XT[] = {delta.Xa, delta.Xb, delta.Xc};
    float YT[] = {delta.Ya, delta.Yb, delta.Yc};
    float ZT[] = {delta.Za, delta.Zb, delta.Zc};
    for (int i = 0; i < 3; ++i) {
      turquoise.active();  
      float dx = sinAngle[i] * delta.JointsDistance / 2;
      float dy = cosAngle[i] * delta.JointsDistance / 2;
      float dex = cosAngle[i] * delta.EffectorOffset;
      float dey = sinAngle[i] * delta.EffectorOffset;
      glBegin(GL_LINES);
        glVertex3f(delta.x + dex + dx, delta.y + dey - dy, delta.z + delta.NozzleHeight);
        glVertex3f(XT[i] + dx, YT[i] - dy, ZT[i]);
      glEnd();
      glBegin(GL_LINES);
        glVertex3f(delta.x + dex - dx, delta.y + dey + dy, delta.z + delta.NozzleHeight);
        glVertex3f(XT[i] - dx, YT[i] + dy, ZT[i]);
      glEnd();
      float ddx = delta.x + dex - XT[i];
      float ddy = delta.y + dey - YT[i];
      float ddz = delta.z + delta.NozzleHeight - ZT[i];
      float l = sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
      printf("l[%d] = %f]\n", i, l);
      // draw joints
      gold.active();
      glPushMatrix();
        glTranslatef(delta.x + dex + dx, delta.y + dey - dy, delta.z + delta.NozzleHeight);
        drawSphere(5, 10, 10);
        glTranslatef(-2 * dx, 2 * dy,0);
        drawSphere(5, 10, 10);
      glPopMatrix();
      glPushMatrix();
        glTranslatef(XT[i] + dx, YT[i] - dy, ZT[i]);
        drawSphere(5, 10, 10);
        glTranslatef(-2 * dx, 2 * dy,0);
        drawSphere(5, 10, 10);
      glPopMatrix();            
    }
  }

  void drawEffector() {
    glPushMatrix();
      glTranslatef(delta.x, delta.y, delta.z + delta.NozzleHeight);
      brass.active();
      glBegin(GL_TRIANGLES);
        glVertex3f(-sqrt(3) * delta.EffectorOffset, delta.EffectorOffset, 0);
        glVertex3f(0, - 2 * delta.EffectorOffset, 0);
        glVertex3f(sqrt(3) * delta.EffectorOffset, delta.EffectorOffset, 0);
      glEnd();
      ruby.active();
      drawCircle(delta.EffectorOffset);
    glPopMatrix();
  }

  void drawNozzle() {
    gold.active();
    glPushMatrix();
      glTranslatef(delta.x, delta.y, delta.z);
      drawCone(delta.NozzleHeight, 0, 6);
    glPopMatrix();    
  }
*/

private:
  DeltaPrinter& delta;
  int renderMode;
  GLuint tex;
  int8_t* texDat;
};

#endif