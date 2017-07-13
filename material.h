#ifndef _MATERIAL_H
#define _MATERIAL_H
#include <GL/gl.h>

class material{

  private:

    GLfloat ambient_r;
    GLfloat ambient_g;
    GLfloat ambient_b;
    GLfloat ambient_a;

    GLfloat diffuse_r;
    GLfloat diffuse_g;
    GLfloat diffuse_b;

    GLfloat specular_r;
    GLfloat specular_g;
    GLfloat specular_b;
    GLfloat shine;


public:

    material(GLfloat ar, GLfloat ag, GLfloat ab, GLfloat aa, GLfloat dr, GLfloat dg, GLfloat db, GLfloat sr, GLfloat sg, GLfloat sb, GLfloat sh){
        ambient_r=ar;   ambient_g=ag;   ambient_b=ab; GLfloat ambient_a = aa;
        diffuse_r=dr;   diffuse_g=dg;   diffuse_b=db;
        specular_r=sr;  specular_g=sg;  specular_b=sb;
        shine = sh;
    }

    void active(){
        GLfloat ambient[]={ambient_r, ambient_g, ambient_b, ambient_a};
        glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
        GLfloat diffuse[]={diffuse_r, diffuse_g, diffuse_b,1.0};
        glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
        GLfloat specular[]={specular_r, specular_g, specular_b,1.0};
        glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
        glMaterialf(GL_FRONT, GL_SHININESS, shine * 128.0);
    }
};

material turquoise(0.1, 0.18725, 0.1745, 1, 0.396, 0.74151, 0.69102, 0.297254, 0.30829, 0.306678, 0.1);
material emerald(0.0215, 0.1745, 0.0215, 1, 0.07568, 0.61424, 0.07568, 0.633, 0.727811, 0.633, 0.6);
material jade(0.135, 0.2225, 0.1575, 1, 0.54, 0.89, 0.63, 0.316228, 0.316228, 0.316228, 0.1);
material obsidian(0.05375, 0.05, 0.06625, 1, 0.18275, 0.17, 0.22525, 0.332741, 0.328634, 0.346435, 0.3);
material chrome(0.25,    0.25,    0.25,  1,   0.4, 0.4, 0.4, 0.774597,    0.774597,    0.774597,    0.6);
material brass(0.329412,    0.223529,    0.027451, 1,    0.780392,    0.568627,    0.113725,    0.992157,    0.941176,    0.807843,    0.21794872);
material gold(0.24725, 0.1995,  0.0745, 1,  0.75164, 0.60648, 0.22648, 0.628281,    0.555802,    0.366065,    0.4);
material whitePlastic(0.0,   0.0, 0.0, 0.55, 1,     0.55,    0.55,    0.70,    0.70,    0.70,    .25);
material ruby( 0.1745f, 0.01175f, 0.01175f ,0.55f, 0.61424f, 0.04136f, 0.04136f, 0.727811f, 0.626959f, 0.626959f, 0.6);
material greenRubber(0.0, 0.05,    0.0, 1, 0.4, 0.5, 0.4, 0.04,    0.7, 0.04,   0.078125);
#endif