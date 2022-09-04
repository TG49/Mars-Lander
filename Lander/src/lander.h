// Mars lander simulator
// Version 1.11
// Header file
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#ifndef LANDER
#define LANDER
#ifdef _WIN32
#define _USE_MATH_DEFINES
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#define NDEBUG
#include <GL/glut.h>
#endif

#include "TextureObject.h"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <Eigen/Dense>
#include <vector>


// GLUT mouse wheel operations work under Linux only
#if !defined (GLUT_WHEEL_UP)
#define GLUT_WHEEL_UP 3
#define GLUT_WHEEL_DOWN 4
#endif

// Graphics constants
#define GAP 5
#define SMALL_NUM 0.0000001
#define N_RAND 20000
#define PREFERRED_WIDTH 1024
#define PREFERRED_HEIGHT 768
#define MIN_INSTRUMENT_WIDTH 1024
#define INSTRUMENT_HEIGHT 300
#define GROUND_LINE_SPACING 20.0
#define CLOSEUP_VIEW_ANGLE 30.0
#define TRANSITION_ALTITUDE 10000.0
#define TRANSITION_ALTITUDE_NO_TEXTURE 4000.0
#define TERRAIN_TEXTURE_SIZE 1024
#define INNER_DIAL_RADIUS 65.0
#define OUTER_DIAL_RADIUS 75.0
#define MAX_DELAY 160000
#define N_TRACK 1000
#define TRACK_DISTANCE_DELTA 100000.0
#define TRACK_ANGLE_DELTA 0.999
#define HEAT_FLUX_GLOW_THRESHOLD 1000000.0
#define MAX_SURFACE_ALTITUDE 2500.0
#define MIN_SURFACE_ALTITUDE -400.0

// Mars constants
#define MARS_RADIUS 3386000.0 // (m)
#define MARS_MASS 6.42E23 // (kg)
#define GRAVITY 6.673E-11 // (m^3/kg/s^2)
#define MARS_DAY 88642.65 // (s)
#define EXOSPHERE 200000.0 // (m)

// Lander constants
#define LANDER_SIZE 1.0 // (m)
#define UNLOADED_LANDER_MASS 100.0 // (kg)
#define FUEL_CAPACITY 100.0 // (l)
#define FUEL_RATE_AT_MAX_THRUST 0.5 // (l/s)
#define FUEL_DENSITY 1.0 // (kg/l)
#define NUM_CHUTES 5.0
// MAX_THRUST, as defined below, is 1.5 * weight of fully loaded lander at surface
#define MAX_THRUST (1.5 * (FUEL_DENSITY*FUEL_CAPACITY+UNLOADED_LANDER_MASS) * (GRAVITY*MARS_MASS/(MARS_RADIUS*MARS_RADIUS))) // (N)
#define ENGINE_LAG 0.0 // (s)
#define ENGINE_DELAY 0.0 // (s)
#define DRAG_COEF_CHUTE 2.0
#define DRAG_COEF_LANDER 1.0
#define MAX_PARACHUTE_DRAG 20000.0 // (N)
#define MAX_PARACHUTE_SPEED 500.0 // (m/s)
#define THROTTLE_GRANULARITY 20 // for manual control
#define MAX_IMPACT_GROUND_SPEED 1.0 // (m/s)
#define MAX_IMPACT_DESCENT_RATE 1.0 // (m/s)

using namespace std;


// Data type for recording lander's previous positions
struct track_t {
  unsigned short n;
  unsigned short p;
  Eigen::Vector3d pos[N_TRACK];
};

// Quaternions for orbital view transformation
struct quat_t {
  Eigen::Vector3d v;
  double s;
};

// Data structure for the state of the close-up view's coordinate system
struct closeup_coords_t {
  bool initialized;
  bool backwards;
  Eigen::Vector3d right;
};

// Enumerated data type for parachute status
enum parachute_status_t { NOT_DEPLOYED = 0, DEPLOYED = 1, LOST = 2 };

#ifdef DECLARE_GLOBAL_VARIABLES // actual declarations of all global variables for lander_graphics.cpp

// GL windows and objects
int main_window, closeup_window, orbital_window, instrument_window, view_width, view_height, win_width, win_height;
GLuint terrain_texture;
short throttle_control;
track_t track;
bool texture_available;

// Simulation parameters
bool help = false;
bool paused = false;
bool landed = false;
bool crashed = false;
int last_click_x = -1;
int last_click_y = -1;
short simulation_speed = 5;
double delta_t, simulation_time;
unsigned short scenario = 0;
string scenario_description[10];
bool static_lighting = false;
closeup_coords_t closeup_coords;
float randtab[N_RAND];
bool do_texture = true;
unsigned long throttle_buffer_length, throttle_buffer_pointer;
double *throttle_buffer = NULL;
unsigned long long time_program_started;
bool simulationInitialized;

// Lander state - the visualization routines use velocity_from_positions, so not sensitive to 
// any errors in the velocity update in numerical_dynamics
Eigen::Vector3d position, orientation, velocity, velocity_from_positions, last_position, positionOnSurface;
double climb_speed, ground_speed, altitude, throttle, fuel, distanceToTerrain;
bool autopilot_enabled, parachute_lost;
parachute_status_t parachute_status;
int stabilized_attitude_angle;

double angularPitchVelocity;
double angularYawVelocity;
double rotationArray[16];
bool Initialised;
Eigen::Quaterniond rotQuat;
bool alignToVelocity;
bool alignToPosition;
bool startOnSurface;

//Texture

textureObject planet;
textureObject surface;
textureObject lowResMars;
heightMapTexture marsHeight;

GLUquadric* qobj;

//Planar Mesh
std::vector<Eigen::Vector2d> vertices; //2D as the y-axis changes with time
std::vector<int> indices;
std::vector<Eigen::Vector2d> texCoords;
std::vector<double> randHeight;




// Orbital and closeup view parameters
double orbital_zoom, save_orbital_zoom, closeup_offset, closeup_xr, closeup_yr, terrain_angle;
quat_t orbital_quat;

// For GL lights
GLfloat plus_y[] = { 0.0, 1.0, 0.0, 0.0 };
GLfloat minus_y[] = { 0.0, -1.0, 0.0, 0.0 };
GLfloat plus_z[] = { 0.0, 0.0, 1.0, 0.0 };
GLfloat top_right[] = { 1.0, 1.0, 1.0, 0.0 };
GLfloat straight_on[] = { 0.0, 0.0, 1.0, 0.0 };

#else // extern declarations of those global variables used in lander.cpp

extern bool autopilot_enabled;
extern double delta_t, simulation_time, throttle, fuel;
extern unsigned short scenario;
extern string scenario_description[];
extern Eigen::Vector3d position, orientation, velocity;
extern parachute_status_t parachute_status;
extern int stabilized_attitude_angle;
extern double angularPitchVelocity;
extern double angularYawVelocity;
extern bool alignToVelocity;
extern bool alignToPosition;
extern bool startOnSurface;

#endif

#ifdef EXTENSION
	extern double landerArea = M_PI * LANDER_SIZE * LANDER_SIZE;
	extern double parachuteArea = (2 * LANDER_SIZE) * (2 * LANDER_SIZE);
	extern double rotationArray[16];
	extern bool Initialised;
	extern Eigen::Quaterniond rotQuat;
#endif

// Function prototypes for definition in lander_graphics
void invert (double m[], double mout[]);
void xyz_euler_to_matrix (Eigen::Vector3d ang, double m[]);
Eigen::Vector3d matrix_to_xyz_euler (double m[]);
void normalize_quat (quat_t &q);
quat_t axis_to_quat (Eigen::Vector3d a, const double phi);
double project_to_sphere (const double r, const double x, const double y);
quat_t add_quats (quat_t q1, quat_t q2);
void quat_to_matrix (double m[], const quat_t q);
quat_t track_quats (const double p1x, const double p1y, const double p2x, const double p2y);
void microsecond_time (unsigned long long &t);
void fghCircleTable (double **sint, double **cost, const int n);
void glutOpenHemisphere (GLdouble radius, GLint slices, GLint stacks);
void drawSphere (GLdouble radius, GLint slices, GLint stacks, GLuint texture);
void glutCone (GLdouble base, GLdouble height, GLint slices, GLint stacks, bool closed);
void enable_lights (void);
void setup_lights (void);
void glut_print (float x, float y, string s);
double atmospheric_density (Eigen::Vector3d pos);
void draw_dial (double cx, double cy, double val, string title, string units);
void draw_control_bar (double tlx, double tly, double val, double red, double green, double blue, string title);
void draw_indicator_lamp (double tcx, double tcy, string off_text, string on_text, bool on);
void draw_instrument_window (void);
void display_help_arrows (void);
void display_help_prompt (void);
void display_help_text (void);
void draw_orbital_window (void);
void draw_parachute_quad (double d);
void draw_parachute (double d);
void update_closeup_coords (void);
void draw_closeup_window (void);
void draw_main_window (void);
void refresh_all_subwindows (void);
bool safe_to_deploy_parachute (void);
void update_visualization (void);
void AlignToVector(Eigen::Vector3d vector);
Eigen::Vector3d thrust_wrt_world (void);
void autopilot (void);
void numerical_dynamics (void);
void initialize_simulation (void);
void update_lander_state (void);
void reset_simulation (void);
void set_orbital_projection_matrix (void);
void reshape_main_window (int width, int height);
void orbital_mouse_button (int button, int state, int x, int y);
void orbital_mouse_motion (int x, int y);
void closeup_mouse_button (int button, int state, int x, int y);
void closeup_mouse_motion (int x, int y);
void glut_special (int key, int x, int y);
void glut_key (unsigned char k, int x, int y);
void loadCloseUpTextures(textureObject& planet, textureObject& surface, heightMapTexture& heightMap);
void loadOrbitalTextures(textureObject& orbital);
void buildPlanarMesh(int numTextureRepeats, int meshResolution, std::vector<Eigen::Vector2d> &vertices,
	std::vector<int> &indices, std::vector<Eigen::Vector2d> &texCoords);
void getPositionalUVCoordinates(double &u, double &v);
#endif