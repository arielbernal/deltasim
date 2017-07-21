#ifndef _DELTA_H_
#define _DELTA_H_

#include <cmath>
#include "svector.h"

static float toRad(float deg) {
  return deg * M_PI / 180;
}

struct DeltaGeometry {
  // Geometry description ------------------------------------------------------------------------
  float DeltaRadius = 180.0f;     // from carriage joint to center (of the printer)
  float ZMax = 800;               // from bed to end-stop
  float Alpha = 90;               // angle from x-axis to A tower
  float Beta = 210;               // angle from x-axis to B tower
  float Gamma = 330;              // angle from x-axis to C tower 
  float RodLength = 360;          // rod length from joint to joint
  float EffectorOffset = 35;      // from center of the effector to side (center of joints)
  float RodRadius = 180;          // from carriage joint to effector joint
  float NozzleHeight = 40;        // from tip of the hot-end nozzle to center of the effector plane
  float RadiusCorrectionA = 0;    // Delta radius correction A tower
  float RadiusCorrectionB = 0;    // Delta radius correction B tower
  float RadiusCorrectionC = 0;    // Delta radius correction C tower
  float DiagonalCorrectionA = 0;  // Diagonal correction A tower
  float DiagonalCorrectionB = 0;  // Diagonal correction B tower
  float DiagonalCorrectionC = 0;  // Diagonal correction C tower
  float TowerAOffset = 0;         // Tower A offset
  float TowerBOffset = 0;         // Tower B offset
  float TowerCOffset = 0;         // Tower C offset
};

struct DeltaPrinter {
/*  =========== Parameter essential for delta calibration ===================

            C, Y-Axis
            |                        |___| CARRIAGE_HORIZONTAL_OFFSET (recommend set it to 0)
            |                        |   \------------------------------------------
            |_________ X-axis        |    \                                        |
           / \                       |     \  DELTA_DIAGONAL_ROD (length)    Each move this Rod Height
          /   \                             \                                 is calculated
         /     \                             \    Carriage is at printer center!   |
         A      B                             \_____/--------------------------------
                                              |--| END_EFFECTOR_HORIZONTAL_OFFSET (recommend set it to 0)
                                         |----| ROD_RADIUS (Horizontal rod pivot to pivot measure)
                                     |-----------| PRINTER_RADIUS (recommend set it to ROD_RADIUS)

    Column angles are measured from X-axis counterclockwise
    "Standard" positions: alpha_A = 210, alpha_B = 330, alpha_C = 90
*/

  DeltaGeometry geo;   // Geometry 
  DeltaGeometry tgeo;  // Temporary geometry
  DeltaPrinter() {}


  static const int sm_width = 1024;
  static const int sm_height = 1024;
  float smx[sm_height * sm_width];
  float smy[sm_height * sm_width];
  float smz[sm_height * sm_width];
  float smxy[sm_height * sm_width];

  void surfaceMap() {
    float DeltaRadius = 180.0f;
    float W = DeltaRadius;
    float x0 = -W;
    float y0 = -W;
    float dx =  2 * W / (sm_width - 1);
    float dy =  2 * W / (sm_height - 1);

    //tgeo.RodLength = geo.RodLength - 0.2;
    tgeo.TowerAOffset = geo.TowerAOffset - 3;

    for (int i = 0; i < sm_height; ++i) {
      for (int j = 0; j < sm_width; ++j) {
        float xx = x0 + dx * j;
        float yy = y0 + dy * i;
        float zz = 0;
        float za = 0, zb = 0, zc = 0;
        float tx = 0, ty = 0, tz = 0;
        inverseKinematics(xx, yy, zz, za, zb, zc, tgeo); 
        forwardKinematics(za, zb, zc, tx, ty, tz, geo);
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


  void inverseKinematics(float x, float y, float z, float &Za, float &Zb, float &Zc, DeltaGeometry& geo) {
    float RLa = geo.RodLength + geo.DiagonalCorrectionA;
    float RLb = geo.RodLength + geo.DiagonalCorrectionB;
    float RLc = geo.RodLength + geo.DiagonalCorrectionC;
    float Ra = geo.RodRadius + geo.RadiusCorrectionA;
    float Rb = geo.RodRadius + geo.RadiusCorrectionB;
    float Rc = geo.RodRadius + geo.RadiusCorrectionC;
    float rAlpha = toRad(geo.Alpha);
    float rBeta = toRad(geo.Beta);
    float rGamma = toRad(geo.Gamma);
    float rax = cos(rAlpha) * Ra - x;
    float ray = sin(rAlpha) * Ra - y;
    float rbx = cos(rBeta)  * Rb - x;
    float rby = sin(rBeta)  * Rb - y;
    float rcx = cos(rGamma) * Rc - x;
    float rcy = sin(rGamma) * Rc - y;
    Za = z + geo.TowerAOffset + sqrt(RLa * RLa - rax * rax - ray * ray);
    Zb = z + geo.TowerBOffset + sqrt(RLb * RLb - rbx * rbx - rby * rby);    
    Zc = z + geo.TowerCOffset + sqrt(RLc * RLc - rcx * rcx - rcy * rcy);
  }

  void forwardKinematics(float Za, float Zb, float Zc, float &x, float &y, float &z, DeltaGeometry& geo) {
    // Using real geometry for forward kinematics
    using namespace svector;
    float rAlpha = toRad(geo.Alpha);
    float rBeta = toRad(geo.Beta);
    float rGamma = toRad(geo.Gamma);
    float RLa = geo.RodLength + geo.DiagonalCorrectionA;
    float RLb = geo.RodLength + geo.DiagonalCorrectionB;
    float RLc = geo.RodLength + geo.DiagonalCorrectionC;
    float Ra = geo.RodRadius + geo.RadiusCorrectionA;
    float Rb = geo.RodRadius + geo.RadiusCorrectionB;
    float Rc = geo.RodRadius + geo.RadiusCorrectionC;

    float Xa = cos(rAlpha) * Ra;
    float Ya = sin(rAlpha) * Ra;
    float Xb = cos(rBeta) * Rb;
    float Yb = sin(rBeta) * Rb;
    float Xc = cos(rGamma) * Rc;
    float Yc = sin(rGamma) * Rc;

    float4 P1(Xa, Ya, Za + geo.TowerAOffset);
    float4 P2(Xb, Yb, Zb + geo.TowerBOffset);
    float4 P3(Xc, Yc, Zc + geo.TowerCOffset);
    
    float d = norm(P2 - P1);
    float4 ex = (P2 - P1) / d;
    float i = dot3d(ex, P3 - P1);
    float4 iex = i * ex;
    float4 ey = (P3 - P1 - iex) / norm(P3 - P1 - iex);
    float j = dot3d(ey, P3 - P1);
    float4 ez = cross3d(ex, ey);

    float xx = (RLa * RLa - RLb * RLb + d * d) / (2 * d);
    float yy = (RLa * RLa - RLc * RLc + i * i + j * j - 2 * i * xx) / (2 * j);
    float zz = sqrt(RLa * RLa - xx * xx - yy * yy);

    float4 p = P1 + xx * ex + yy * ey - zz * ez;
    x = p.x; y = p.y; z = p.z;
  }

};

#endif