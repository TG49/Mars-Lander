#ifndef EXTENSION
#define EXTENSION
#include "lander.h"
#include <Eigen/dense>
#include <vector>


//Function Declarations for lander.cpp
Eigen::Vector3d updateAccelerationVector(double& mass);
Eigen::Vector3d updateGravitationVector();
Eigen::Vector3d updateThrustVector(double& mass);
Eigen::Vector3d updateDragVector(double& mass);


void updateMass(double& mass);
void Euler();
void Verlet();
void adjustAttitude();
void logTelemetry(double altitude);

bool parachuteSafeToDeploy(double altitude);
double findAutopilotThrottle();
double desiredVelocity(double altitude);



Eigen::Matrix4d quaternionRotationMatrix(double pitch, double yaw, double roll, std::vector<Eigen::Vector3d> axes);

#endif