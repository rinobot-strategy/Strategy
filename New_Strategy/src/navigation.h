#ifndef UTILS_H
#define UTILS_H

#include "iostream"
#include "math.h"
#undef Sucess
#include <eigen3/Eigen/Dense>
#define pi M_PI
#include "common.h"

using namespace common;
using namespace Eigen;
using namespace std;

class navigation{
public:
navigation();
~navigation();
string side;
btVector3 centroid_atk, centroid_def; 
float hyperbolic_spiral(float yi, float xi, btVector3 meta);
void generate_univector(float yi, float xi, Robot robo, btVector3 meta, btVector3 enemy);
void set_theta_dir(float);
float Gaussian_Func(float r);
float repulsive_angle(float y, float x, btVector3 pos);
float tangencial_repulsive(Robot robo, btVector3 meta, btVector3 obstaculo, float r);
float repulsive_spiral(Robot robo, btVector3 enemy);
float the_fih, theta_dir, omega;
void set_side(string);
void fake_cph(Robot robo, btVector3 meta);
float get_angle_cpu();
float get_angle();
btVector3 meta_fake_cph;
};

#endif