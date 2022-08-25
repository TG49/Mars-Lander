#ifndef EXTENSION
#define EXTENSION
#include "lander.h"

//Function Declarations for lander.cpp
vector3d updateAccelerationVector(double& mass);
vector3d updateGravitationVector();
vector3d updateThrustVector(double& mass);
vector3d updateDragVector(double& mass);



void updateMass(double& mass);
void Euler();
void Verlet();
void adjustAttitude();

#endif