// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include "extension.h"
#include <vector>
#include <Eigen/dense>

//Update the acceleration vector associated with the lander craft.

/// <summary>
/// Function to use for the autopilot
/// </summary>
/// <param name=""></param>
void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    throttle = findAutopilotThrottle();
}


/// <summary>
/// Computes the integrator for the next timestep
/// </summary>
void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
        if (alignToPosition) {
            AlignToVector(position);
        }
        else if (alignToVelocity) {
            AlignToVector(velocity);
        }
        else if (alignToNegativeVelocity) {
            AlignToVector(-1 * velocity);
        }
        else {
            adjustAttitude();
        }

    //Euler();
    Verlet();
 
  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

}


/// <summary>
/// Initialise the simulation
/// </summary>
void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen
  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "Geosynchronous Orbit";
  scenario_description[7] = "High Altitude Vertical Descent. Maximum landable alitude with current no parachute autopilot";
  scenario_description[8] = "High Altitude Vertical Descent. Maximum landable alitude with current parachute autopilot";
  scenario_description[9] = "";

  startOnSurface = false;

  double geostationaryRadiusCubed = (MARS_DAY) * (MARS_DAY)*GRAVITY * MARS_MASS / (4 * M_PI * M_PI);
  double geostationaryRadius = std::pow(geostationaryRadiusCubed, 1.0 / 3.0);
  double geostationaryVelocityMagnitude = std::sqrt(GRAVITY * MARS_MASS / geostationaryRadius);

  alignToVelocity = false;
  alignToPosition = false;
  alignToNegativeVelocity = false;
  autopilot_enabled = false;

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = Eigen::Vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = Eigen::Vector3d(0.0, -3247.087385863725, 0.0);
    orientation = Eigen::Vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = Eigen::Vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    orientation = Eigen::Vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    break;

  case 2:
    // an elliptical polar orbit
    position = Eigen::Vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = Eigen::Vector3d(3500.0, 0.0, 0.0);
    orientation = Eigen::Vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    startOnSurface = true;
    position = Eigen::Vector3d(0.0, 0.0, MARS_RADIUS); //Not really relevant
    velocity = Eigen::Vector3d(0.0, 0.0, 0);
    orientation = Eigen::Vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = Eigen::Vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = Eigen::Vector3d(4000.0, 0.0, 0.0);
    orientation = Eigen::Vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = Eigen::Vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    orientation = Eigen::Vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    break;

  case 6:
      // A Geostationary orbit

      position = Eigen::Vector3d(geostationaryRadius, 0, 0);
      velocity = Eigen::Vector3d(0, geostationaryVelocityMagnitude, 0);
      orientation = Eigen::Vector3d(0.0, 90.0, 0.0);
      delta_t = 0.1;
      parachute_status = NOT_DEPLOYED;
      break;
  case 7:
      //High Altitude Vertical Descent. Maximum landable alitude with current autopilot
      position = Eigen::Vector3d(0.0, -(MARS_RADIUS +605000), 0.0);
      velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
      orientation = Eigen::Vector3d(0.0, 0.0, 90.0);
      delta_t = 0.1;
      parachute_status = NOT_DEPLOYED;
      break;

  case 8:
      //High Altitude Vertical Descent. Maximum landable alitude with current autopilot
      position = Eigen::Vector3d(0.0, -(MARS_RADIUS + 1260000), 0.0);
      velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
      orientation = Eigen::Vector3d(0.0, 0.0, 90.0);
      delta_t = 0.1;
      parachute_status = NOT_DEPLOYED;
      break;

  case 9:
    break;

  }
}
