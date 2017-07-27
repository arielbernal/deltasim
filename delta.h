#ifndef _DELTA_H_
#define _DELTA_H_

#include <cmath>
#include "svector.h"

static float toRad(float deg) {
  return deg * M_PI / 180;
}

struct DeltaGeometry {
  // Geometry description ------------------------------------------------------------------------
  float RodRadius = 174.50;       // from carriage joint to effector joint  
  float Alpha = 90;               // angle from x-axis to A tower
  float Beta = 210;               // angle from x-axis to B tower
  float Gamma = 330;              // angle from x-axis to C tower 
  float RodLength = 360.40;       // rod length from joint to joint

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

  DeltaPrinter() {}

  static const int sm_width = 1024;
  static const int sm_height = 1024;
  float smx[sm_height * sm_width];
  float smy[sm_height * sm_width];
  float smz[sm_height * sm_width];
  float smxy[sm_height * sm_width];



  void setGeo(DeltaGeometry& Geo) {
    // Geo.TowerAOffset = -1;
    // Geo.TowerBOffset = 2.5;
    // Geo.TowerCOffset = -1.5;
    // Geo.RodRadius = 158;
    Geo.RodLength = 363;
  }

  void surfaceMap(DeltaGeometry& IKGeo, DeltaGeometry& FKGeo) {
    float DeltaRadius = FKGeo.RodRadius;
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
        inverseKinematics(xx, yy, zz, za, zb, zc, IKGeo); 
        forwardKinematics(za, zb, zc, tx, ty, tz, FKGeo);
        int idx = (sm_height - i - 1) * sm_width + j;
        if (sqrt(xx * xx +  yy * yy) >= DeltaRadius)  {
          smx[idx] = 30000;
          smy[idx] = 30000;
          smz[idx] = 30000;
          smxy[idx] = 30000;
        } else {
          smx[idx] = (tx - xx);
          smy[idx] = (ty - yy);
          smxy[idx] = sqrt(smx[idx] * smx[idx] + smy[idx] * smy[idx]);
          smz[idx] = tz - zz;
        }
      }
    }
  }


  void calibrate1(DeltaGeometry& IKGeo, DeltaGeometry& FKGeo) {
    using namespace svector;
    float Alpha = 90;
    float Beta = 210;
    float Gamma = 330;
    float r = 160;
    float4 r0(0, 0, 0);
    float4 rA0(r * cos(toRad(Alpha)), r * sin(toRad(Alpha)), 0);
    float4 rA1(-r * cos(toRad(Alpha)), -r * sin(toRad(Alpha)), 0);
    float4 rB0(r * cos(toRad(Beta)), r * sin(toRad(Beta)), 0);
    float4 rB1(-r * cos(toRad(Beta)), -r * sin(toRad(Beta)), 0);
    float4 rC0(r * cos(toRad(Gamma)), r * sin(toRad(Gamma)), 0);
    float4 rC1(-r * cos(toRad(Gamma)), -r * sin(toRad(Gamma)), 0);

    float k = IKGeo.RodRadius / r;
    float ZA0 = (FK(IK(rA0, IKGeo), FKGeo) - rA0).z;
    float ZA1 = (FK(IK(rA1, IKGeo), FKGeo) - rA1).z;
    float DZA = (ZA0 * (1 + k) + ZA1 * (1 - k)) / 2;

    float ZB0 = (FK(IK(rB0, IKGeo), FKGeo) - rB0).z;
    float ZB1 = (FK(IK(rB1, IKGeo), FKGeo) - rB1).z;
    float DZB = (ZB0 * (1 + k) + ZB1 * (1 - k)) / 2;

    float ZC0 = (FK(IK(rC0, IKGeo), FKGeo) - rC0).z;
    float ZC1 = (FK(IK(rC1, IKGeo), FKGeo) - rC1).z;
    float DZC = (ZC0 * (1 + k) + ZC1 * (1 - k)) / 2;

    float ZR0 = (FK(IK(r0, IKGeo), FKGeo) - r0).z;
    float ZT = (ZA0 + ZA1 + ZB0 + ZB1 + ZC0 + ZC1) / 6;

    printf("--------------------------------------------\n");
    printf("IKGeo.TowerAOffset = %f, ZA0 = %f, ZA1 = %f, DZA = %f\n", IKGeo.TowerAOffset, ZA0, ZA1, DZA);
    printf("IKGeo.TowerBOffset = %f, ZB0 = %f, ZB1 = %f, DZB = %f\n", IKGeo.TowerBOffset, ZB0, ZB1, DZB);
    printf("IKGeo.TowerCOffset = %f, ZC0 = %f, ZC1 = %f, DZC = %f\n", IKGeo.TowerCOffset, ZC0, ZC1, DZC);
    printf("IKGeo.RodRadius = %f,    ZR0 = %f\n", IKGeo.RodRadius, ZR0);
    printf("IKGeo.RodLength = %f,    ZT = %f\n", IKGeo.RodLength, ZT);
    IKGeo.TowerAOffset += DZA;
    IKGeo.TowerBOffset += DZB;
    IKGeo.TowerCOffset += DZC;

    
    IKGeo.RodLength -= ZT;
    if (fabs(ZT) < 0.1) {
      IKGeo.RodRadius += 1.5 * ZR0;
    }


  }



  float cost(DeltaGeometry& IKGeo, DeltaGeometry& FKGeo, svector::float4* v, float* z, int N) {
      float cost = 0;
      for (int i = 0; i < N; ++i) {
        float d = FK(IK(v[i], IKGeo), FKGeo).z - z[i];
        cost += d * d;
      }
      return sqrt(cost) / N;
  }

  float randf() {
    return (float)rand() / (RAND_MAX + 1.0f);
  }

  DeltaGeometry geoNeighbour(DeltaGeometry& geo) {
    int geoField = rand() % 4;
    int change = rand() % 2;
    DeltaGeometry t = geo;
    switch(geoField) {
      case 0 : t.TowerAOffset += (change ? 0.1 : -0.1); break;
      case 1 : t.TowerBOffset += (change ? 0.1 : -0.1); break;
      case 2 : t.TowerCOffset += (change ? 0.1 : -0.1); break;
      // case 3 : t.RodRadius *= change ? 0.9 : 1.1; break;
      // case 4 : t.Alpha *= change ? 0.9 : 1.1; break;
      // case 5 : t.Beta *= change ? 0.9 : 1.1; break;
      // case 6 : t.Gamma *= change ? 0.9 : 1.1; break;
      // case 7 : t.RadiusCorrectionA *= change ? 0.9 : 1.1; break;
      // case 8 : t.RadiusCorrectionB *= change ? 0.9 : 1.1; break;
      // case 9 : t.RadiusCorrectionC *= change ? 0.9 : 1.1; break;

    }
    return t;
  }

  void calibrate(DeltaGeometry& IKGeo, DeltaGeometry& FKGeo) {
    using namespace svector;
    float Alpha = 90;
    float Beta = 210;
    float Gamma = 330;
    float r = 160;
    static const int N = 7;
    float4 v[N];
    v[0] = float4(0, 0, 0);
    v[1] = float4(r * cos(toRad(Alpha)), r * sin(toRad(Alpha)), 0);
    v[2] = float4(-r * cos(toRad(Alpha)), -r * sin(toRad(Alpha)), 0);
    v[3] = float4(r * cos(toRad(Beta)), r * sin(toRad(Beta)), 0);
    v[4] = float4(-r * cos(toRad(Beta)), -r * sin(toRad(Beta)), 0);
    v[5] = float4(r * cos(toRad(Gamma)), r * sin(toRad(Gamma)), 0);
    v[6] = float4(-r * cos(toRad(Gamma)), -r * sin(toRad(Gamma)), 0);
    float z[N] = { 0.3, -0.1, 0.2, -1.2, -0.5, 1.3, 0.3 };

    DeltaGeometry Geo = FKGeo;

    float T = 1E10;
    float TMin = 0.1;
    float TAlpha = 0.99;
    float old_cost = cost(IKGeo, Geo, v, z, N);
    float d = 1e-5;
    printf("Cost = %f\n", old_cost);
    int NIter = 100;
    int itotal = 0;
    while(T > TMin) {
      int i = 0;
      while (i < NIter) {
        DeltaGeometry NewGeo = geoNeighbour(Geo);
        float new_cost = cost(IKGeo, Geo, v, z, N);
        if (new_cost == new_cost) { // check if not nan
          if(new_cost < old_cost || exp(-(new_cost - old_cost) / (d * T)) > randf()) {
            Geo = NewGeo;
            old_cost = new_cost;
            printf("%d Cost = %f T = %f  -  ", itotal, old_cost, T);
            printf("  OA=%f OB=%f OC=%f\n", Geo.TowerAOffset, Geo.TowerBOffset, Geo.TowerCOffset);
          }
          i++;
          itotal++;
        }
      }
      printf("--------------------------------- T = %f\n", T);
      T *= TAlpha;
    }
  }


  svector::float4 IK(const svector::float4 &r, const DeltaGeometry &geo) {
    svector::float4 Z;
    inverseKinematics(r.x, r.y, r.z, Z.x, Z.y, Z.z, geo);
    return Z;
  }

  svector::float4 FK(const svector::float4 &Z, const DeltaGeometry &geo) {
    svector::float4 r;
    forwardKinematics(Z.x, Z.y, Z.z, r.x, r.y, r.z, geo);
    return r;
  }


  void inverseKinematics(float x, float y, float z, float &Za, float &Zb, float &Zc, const DeltaGeometry& geo) {
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
    Za = z - geo.TowerAOffset + sqrt(RLa * RLa - rax * rax - ray * ray);
    Zb = z - geo.TowerBOffset + sqrt(RLb * RLb - rbx * rbx - rby * rby);    
    Zc = z - geo.TowerCOffset + sqrt(RLc * RLc - rcx * rcx - rcy * rcy);
  }

  void forwardKinematics(float Za, float Zb, float Zc, float &x, float &y, float &z, const DeltaGeometry& geo) {
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