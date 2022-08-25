#include "extension.h" //Do not need to include lander.h as it is included in extension.h
#include <vector>

vector3d updateGravitationVector() {
    double distance = position.abs2();
    return (-1 * GRAVITY * MARS_MASS / distance) * position.norm();
}

vector3d updateThrustVector(double& mass) {
    return thrust_wrt_world() / mass;
}

void updateMass(double& mass) {
    //fuel -= delta_t * throttle * FUEL_RATE_AT_MAX_THRUST;
    mass = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;
}

vector3d updateDragVector(double& mass) {
    //Get current atmosphericDensity
    double atmosphericDensity = atmospheric_density(position);

    //Drag on Lander
    vector3d Force = -(0.5) * atmosphericDensity * DRAG_COEF_LANDER * landerArea * velocity.abs2() * velocity.norm();

    //Drag on Parachute
    if (parachute_status == DEPLOYED) {
        double totalChuteArea = NUM_CHUTES * parachuteArea;
        Force += -(0.5) * atmosphericDensity * DRAG_COEF_CHUTE * totalChuteArea * velocity.abs2() * velocity.norm();
    }

    return Force / mass;
}


vector3d updateAccelerationVector(double& mass) {
    updateMass(mass);
    vector3d gravitation = updateGravitationVector();
    vector3d thrust = updateThrustVector(mass);
    vector3d drag = updateDragVector(mass);

    return gravitation + thrust + drag;
}

void Euler() {
    static double mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY;
    vector3d acceleration = updateAccelerationVector(mass);
    position = position + velocity * delta_t;
    velocity = velocity + acceleration * delta_t;
}

void Verlet() {
    static double mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY;
    static std::vector<vector3d> previousStates(2);
    vector3d acceleration = updateAccelerationVector(mass);
    if (simulation_time == 0) {
        previousStates[0] = position;
        position = position + velocity * delta_t;
        velocity = velocity + acceleration * delta_t;
        previousStates[1] = position;
    }
    else {
        position = 2 * position - previousStates[0] + acceleration * delta_t * delta_t;
        velocity = (position - previousStates[1]) / delta_t;
        previousStates[0] = previousStates[1];
        previousStates[1] = position;
    }
}

void adjustAttitude()
// Three-axis stabilization to ensure the lander's base is always pointing downwards 
{
    static vector3d prograde, aRadial, normal; 
    static bool Initialised = false;
    double m[16];

    if (!Initialised)
    {
        prograde = position.norm();
        Initialised = true;
    }
    else {
        if (prograde.x < 0) { normal = -normal; }

        prograde += delta_t * angularPitchVelocity * normal; // change the prograde by a small amount in the normal vector direction.

        prograde += delta_t * angularYawVelocity * aRadial;

        prograde = prograde.norm();



        std::cout << "Prograde: " << prograde << std::endl;
        std::cout << "Angular Pitch Velocity: " << angularPitchVelocity << std::endl;
    }


    // !!!!!!!!!!!!! HINT TO STUDENTS ATTEMPTING THE EXTENSION EXERCISES !!!!!!!!!!!!!!
    // For any-angle attitude control, we just need to set "up" to something different,
    // and leave the remainder of this function unchanged. For example, suppose we want
    // the attitude to be stabilized at stabilized_attitude_angle to the vertical in the
    // close-up view. So we need to rotate "up" by stabilized_attitude_angle degrees around
    // an axis perpendicular to the plane of the close-up view. This axis is given by the
    // vector product of "up"and "closeup_coords.right". To calculate the result of the
    // rotation, search the internet for information on the axis-angle rotation formula.

    // Set left to something perpendicular to up


    //Error. Think it has to do with the Yaw component. The orientation values go 0-90-0 on a 180 degree scale.
    //Having issues where it reads 0. Investigate further.

    aRadial.x = -prograde.y; aRadial.y = prograde.x; aRadial.z = 0.0;
    if (aRadial.abs() < SMALL_NUM) { aRadial.x = -prograde.z; aRadial.y = 0.0; aRadial.z = prograde.x; } //Yaw error here
    aRadial = aRadial.norm();
    cout << "aRadial: " << aRadial << endl;
    normal = aRadial ^ prograde;



    // Construct modelling matrix (rotation only) from these three vectors
    m[0] = normal.x; m[1] = normal.y; m[2] = normal.z; m[3] = 0.0;
    m[4] = aRadial.x; m[5] = aRadial.y; m[6] = aRadial.z; m[7] = 0.0;
    m[8] = prograde.x; m[9] = prograde.y; m[10] = prograde.z; m[11] = 0.0;
    m[12] = 0.0; m[13] = 0.0; m[14] = 0.0; m[15] = 1.0;
    // Decomponse into xyz Euler angles
    orientation = matrix_to_xyz_euler(m);
    cout << "Orientation: " << orientation << endl << endl;
}