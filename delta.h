#ifndef _DELTA_H_
#define _DELTA_H_

#include <cmath>
#include "svector.h"

static float toRad(float deg) {
  return deg * M_PI / 180;
}

struct DeltaPrinter {
  /*
         A           ^
        / \          |
       /   \         y
      /     \        |-----x----->
     B ----- C
  */
  // Geometry description ------------------------------------------------------------------------
  const float DeltaRadius = 180.0f;     // from carriage joint to center (of the printer)
  const float ZMax = 800;               // from bed to end-stop
  const float Alpha = 90;               // angle from x-axis to A tower
  const float Beta = 210;               // angle from x-axis to B tower
  const float Gamma = 330;              // angle from x-axis to C tower 
  const float RodLength = 360;          // rod length from joint to joint
  const float EffectorOffset = 35;      // from center of the effector to side (center of joints)
  const float NozzleHeight = 40;        // from tip of the hot-end nozzle to center of the effector plane

  // Assumed geometric values
  float TDeltaRadius = 180.0f;          // from carriage joint to center (of the printer)
  float TAlpha = 90;                      // angle from x-axis to A tower
  float TBeta = 210;                        // angle from x-axis to B tower
  float TGamma = 330;                      // angle from x-axis to C tower 
  float TRodLength = 360;              // rod length from joint to joint
  float TEffectorOffset = 35;    // from center of the effector to side (center of joints)
  float TNozzleHeight = 40;        // from tip of the hot-end nozzle to center of the effector plane

  // Just for rendering
  const float CarriageOffset = 25.0f;   // from carriage joint to center of the tower
  const float TowerHeight = 1000;       // vertical frame height
  const float FrameBottomH = 60;        // 60x20
  const float FrameBottomW = 20;        //
  const float FrameTopH = 40;           // 40x20
  const float FrameTopW = 20;           //
  const float FrameVerticalH = 40;      // 40x20
  const float FrameVerticalW = 20;      //
  const float FrameVerticalOffset = 10; // center of the tower
  const float BedHeight = 25.3 / 4;     // glass+.. height
  const float JointsDistance = 40;      // distance between parallel rods

  // Deduced constants --------------------------------------------------------------------------
  const float TowerRadius = DeltaRadius + CarriageOffset;

  /*
             /
            /
           / Rod     
      ____/
      |
      v
  */
  float Le;                     // equivalent rod lenght
  // Kinematic variables ------------------------------------------------------------------------
  float Za = ZMax / 2;   // carriage A height
  float Zb = ZMax / 2;   // carriage B height
  float Zc = ZMax / 2;   // carriage C height
  float x, y, z;                // x, y and z of the tip of the nozzle
  // --------------------------------------------------------------------------------------------
  float Xa, Ya;
  float Xb, Yb;
  float Xc, Yc;

  float rAlpha = toRad(Alpha);
  float rBeta = toRad(Beta);
  float rGamma = toRad(Gamma);

  float TrAlpha = toRad(Alpha);
  float TrBeta = toRad(Beta);
  float TrGamma = toRad(Gamma);

  DeltaPrinter() {
    // initialize equivalent rod lenght 
    float Rx = (DeltaRadius - EffectorOffset);
    float Ry = sqrt(RodLength * RodLength - Rx * Rx);
    float Lex = DeltaRadius;
    float Ley = Ry + NozzleHeight;
    Le = sqrt(Lex * Lex + Ley * Ley);
    Xa = DeltaRadius * cos(rAlpha);
    Ya = DeltaRadius * sin(rAlpha);
    Xb = DeltaRadius * cos(rBeta);
    Yb = DeltaRadius * sin(rBeta);
    Xc = DeltaRadius * cos(rGamma);
    Yc = DeltaRadius * sin(rGamma);    
  }


  void setCarriages(float a, float b, float c) {
    Za = a;
    Zb = b;
    Zc = c;
    forwardKinematics(Za, Zb, Zc, x, y, z);
  }

  void moveCarriages(float da, float db, float dc) {
    Za += da;
    Zb += db;
    Zc += dc;
    forwardKinematics(Za, Zb, Zc, x, y, z);
  }

  void setXYZ(float xx, float yy, float zz) {
    x = xx;
    y = yy;
    z = zz;
    inverseKinematics(x, y, z, Za, Zb, Zc);
  }


  static const int sm_width = 1024;
  static const int sm_height = 1024;
  float smx[sm_height * sm_width];
  float smy[sm_height * sm_width];
  float smz[sm_height * sm_width];
  float smxy[sm_height * sm_width];

  void surfaceMap() {
    float W = DeltaRadius;
    float x0 = -W;
    float y0 = -W;
    float dx =  2 * W / (sm_width - 1);
    float dy =  2 * W / (sm_height - 1);

    for (int i = 0; i < sm_height; ++i) {
      for (int j = 0; j < sm_width; ++j) {
        float xx = x0 + dx * j;
        float yy = y0 + dy * i;
        float zz = 0;
        float za = 0, zb = 0, zc = 0;
        float tx = 0, ty = 0, tz = 0;
        inverseKinematics(xx, yy, zz, za, zb, zc); 
        forwardKinematics(za, zb, zc, tx, ty, tz);
        int idx = i * sm_width + j;
        
        
        if (sqrt(xx * xx + yy * yy) >= DeltaRadius)  {
          smx[idx] = 30000;
          smy[idx] = 30000;
          smz[idx] = 30000;
          smxy[idx] = 30000;
        } else {
          smx[idx] = fabs(xx - tx);
          smy[idx] = fabs(yy - ty);
          smxy[idx] = sqrt(smx[idx] * smx[idx] + smy[idx] * smy[idx]);
          smz[idx] = zz - tz;
        }
      }
    }
  }

  void inverseKinematics(float x, float y, float z, float &Za, float &Zb, float &Zc) {
    // Using assumed values for inverse kinematics
    TDeltaRadius = DeltaRadius - 0.1;
    TRodLength = RodLength + 0.5;
    TNozzleHeight = NozzleHeight - 0.5;
    TEffectorOffset = EffectorOffset + 0.3;

    float R2 = TRodLength * TRodLength;
    float rax = cos(TrAlpha) * (TDeltaRadius - TEffectorOffset) - x;
    float ray = sin(TrAlpha) * (TDeltaRadius - TEffectorOffset) - y;
    float rbx = cos(TrBeta) * (TDeltaRadius - TEffectorOffset) - x;
    float rby = sin(TrBeta) * (TDeltaRadius - TEffectorOffset) - y;
    float rcx = cos(TrGamma) * (TDeltaRadius - TEffectorOffset) - x;
    float rcy = sin(TrGamma) * (TDeltaRadius - TEffectorOffset) - y;
    Za = z + TNozzleHeight + sqrt(R2 - rax * rax - ray * ray);
    Zb = z + TNozzleHeight + sqrt(R2 - rbx * rbx - rby * rby);    
    Zc = z + TNozzleHeight + sqrt(R2 - rcx * rcx - rcy * rcy);
  }

  void forwardKinematics(float Za, float Zb, float Zc, float &x, float &y, float &z) {
    // Using real geometry for forward kinematics
    using namespace svector;
    float rax = cos(rAlpha) * EffectorOffset;
    float ray = sin(rAlpha) * EffectorOffset;
    float rbx = cos(rBeta) * EffectorOffset;
    float rby = sin(rBeta) * EffectorOffset;
    float rcx = cos(rGamma) * EffectorOffset;
    float rcy = sin(rGamma) * EffectorOffset;

    float4 P2(Xb - rbx, Yb - rby, Zb);
    float4 P3(Xc - rcx, Yc - rcy, Zc);
    float4 P1(Xa - rax, Ya - ray, Za);
    
    float d = norm(P2 - P1);
    float4 ex = (P2 - P1) / d;
    float i = dot3d(ex, P3 - P1);
    float4 iex = i * ex;
    float4 ey = (P3 - P1 - iex) / norm(P3 - P1 - iex);
    float j = dot3d(ey, P3 - P1);
    float4 ez = cross3d(ex, ey);

    float xx = d / 2;
    float yy = (i * i + j * j - 2 * i * xx) / (2 * j);
    float zz = sqrt(RodLength * RodLength - xx * xx - yy * yy);

    float4 p = P1 + xx * ex + yy * ey - zz * ez;
    x = p.x; y = p.y; z = p.z - NozzleHeight;
  }

};

#endif