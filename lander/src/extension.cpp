#include "extension.h" //Do not need to include lander.h as it is included in extension.h
#include <vector>


Eigen::Vector3d updateGravitationVector() {
    double distance = position.squaredNorm();
    return (-1 * GRAVITY * MARS_MASS / distance) * position.normalized();
}

Eigen::Vector3d updateThrustVector(double& mass) {
    return thrust_wrt_world() / mass;
}

void updateMass(double& mass) {
    //fuel -= delta_t * throttle * FUEL_RATE_AT_MAX_THRUST;
    mass = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;
}

Eigen::Vector3d updateDragVector(double& mass) {
    //Get current atmosphericDensity
    double atmosphericDensity = atmospheric_density(position);

    //Drag on Lander
    Eigen::Vector3d Force = -(0.5) * atmosphericDensity * DRAG_COEF_LANDER * landerArea * velocity.squaredNorm() * velocity.normalized();

    //Drag on Parachute
    if (parachute_status == DEPLOYED) {
        double totalChuteArea = NUM_CHUTES * parachuteArea;
        Force += -(0.5) * atmosphericDensity * DRAG_COEF_CHUTE * totalChuteArea * velocity.squaredNorm() * velocity.normalized();
    }

    return Force / mass;
}


Eigen::Vector3d updateAccelerationVector(double& mass) {
    updateMass(mass);
    Eigen::Vector3d gravitation = updateGravitationVector();
    Eigen::Vector3d thrust = updateThrustVector(mass);
    Eigen::Vector3d drag = updateDragVector(mass);

    return gravitation + thrust + drag;
}

void Euler() {
    static double mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY;
    Eigen::Vector3d acceleration = updateAccelerationVector(mass);
    position = position + velocity * delta_t;
    velocity = velocity + acceleration * delta_t;
}

void Verlet() {
    static double mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY;
    static std::vector<Eigen::Vector3d> previousStates(2);
    Eigen::Vector3d acceleration = updateAccelerationVector(mass);
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


Eigen::Matrix4d quaternionRotationMatrix(double pitch, double yaw, double roll, std::vector<Eigen::Vector3d> axes, Eigen::Quaterniond &currentOrientation) {
    
    ///Check UnitZ here and UnitZ graphical are the same by showing both. 
    Eigen::AngleAxisd rollAngle(roll, axes[2]);
    Eigen::AngleAxisd yawAngle(yaw, axes[1]);
    Eigen::AngleAxisd pitchAngle(pitch, axes[0]);

    Eigen::Quaternion<double> q = pitchAngle * yawAngle * rollAngle;
    q.normalize();

    //Update net rotation matrix
    currentOrientation = q * currentOrientation;
    currentOrientation.normalize();

    Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity(4, 4);
    rotation.block<3, 3>(0, 0) = currentOrientation.toRotationMatrix();
    orientation = currentOrientation.toRotationMatrix().eulerAngles(0,1,2);


    return rotation;
}

void adjustAttitude()
{
    Eigen::Matrix4d rotMatrix4x4 = Eigen::Matrix4d::Identity(4,4); //Rotation matrix
    std::vector<Eigen::Vector3d> axes;

    //Rotate coordinate vectors to align with the current orientation of the spacecraft
    axes.push_back(rotQuat * Eigen::Vector3d::UnitX());
    axes.push_back(rotQuat * Eigen::Vector3d::UnitY());
    axes.push_back(rotQuat * Eigen::Vector3d::UnitZ());

    //Use default orientation for the first call to the function.
    if (!Initialised) {
        orientation = orientation / 180 * M_PI; //Convert orientation from degrees to radians
        rotMatrix4x4 = quaternionRotationMatrix(orientation[2], orientation[1], orientation[0], axes, rotQuat);
        Initialised = true;
    }
    else {
        //Update timestep rotations
        double rollAngle = 0;
        double pitchAngle = angularPitchVelocity * delta_t;
        double yawAngle = angularYawVelocity * delta_t;

        //Find the new rotation matrix which defines the orientation of the craft.
        rotMatrix4x4 = quaternionRotationMatrix(pitchAngle, yawAngle, rollAngle, axes, rotQuat);

        //Save the rotaiton matrix to the rotationArray, which is used by openGL for visualisation
    }
    Eigen::Map<Eigen::Matrix4d>(rotationArray, 0, 0) = rotMatrix4x4;


    /*for (int i = 0; i < size(rotationArray); i++)
    {
        cout << rotationArray[i] << " ";
    }
    cout << endl << endl;*/

}

void velocityAlign(bool alignToVelocity){
    static bool XAligned;
    static bool YAligned;
    static bool VelocityAligned = false;

    if (alignToVelocity) {

        //Get local coordinate system
        Eigen::Vector3d currentOrientation = (rotQuat * Eigen::Vector3d::UnitZ()).normalized();
        Eigen::Vector3d transposedY = (rotQuat * Eigen::Vector3d::UnitY()).normalized();
        Eigen::Vector3d transposedX = (rotQuat * Eigen::Vector3d::UnitZ()).normalized();

        //find normal vector to velocity and orientation
        Eigen::Vector3d normalToOrientationAndVelocity = currentOrientation.cross(velocity.normalized());

        //Test if normal vector parallel to either local X or Y axes.
        if ((normalToOrientationAndVelocity.dot(transposedY) < SMALL_NUM) && (normalToOrientationAndVelocity.dot(transposedX) < SMALL_NUM)) {
            angularPitchVelocity = 0;
            angularYawVelocity = 0;
            VelocityAligned = true;
            cout << "Velocity Aligned " << endl;
            return;
        }
        else if (normalToOrientationAndVelocity.dot(transposedY) < SMALL_NUM)
        {
            YAligned = true;
            XAligned = false;
            angularPitchVelocity = 0;
            cout << "Y Aligned " << endl;
        }
        else if (normalToOrientationAndVelocity.dot(transposedX) < SMALL_NUM) {
            XAligned = true;
            YAligned = false;
            angularPitchVelocity = 0;
            cout << "X Aligned " << endl;
        }
        else {
            //Start rotation. Will continue to rotate until aligned
            angularPitchVelocity = 1;
            cout << "None Aligned" << endl;
            return;
        }

        if (XAligned && !YAligned)
        {
            angularYawVelocity = 1;
            cout << "Increase Yaw Velocity" << endl;
            return;
        }

    }
}