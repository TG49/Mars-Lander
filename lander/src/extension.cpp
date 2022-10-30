#include "extension.h" //Do not need to include lander.h as it is included in extension.h
#include <vector>

/// <summary>
/// Updates the current acceleration vector associated with the force of gravity
/// </summary>
/// <returns>Eigen::Vector3d acceleration vector</returns>
Eigen::Vector3d updateGravitationVector() {
    return (-1 * GRAVITY * MARS_MASS / position.squaredNorm()) * position.normalized();
}

/// <summary>
/// Updates the current acceleration vector associated with thrust
/// </summary>
/// <param name="mass">Mass of spacecraft</param>
/// <returns>Eigen::Vector3d acceleration vector</returns>
Eigen::Vector3d updateThrustVector(double& mass) {
    return thrust_wrt_world() / mass;
}

/// <summary>
/// Updates the mass variable
/// </summary>
/// <param name="mass">current mass variable</param>
void updateMass(double& mass) {
    //fuel -= delta_t * throttle * FUEL_RATE_AT_MAX_THRUST;
    mass = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;
}

/// <summary>
/// Updates the current acceleration vector associated with the atmospheric drag
/// </summary>
/// <param name="mass">current mass</param>
/// <returns>Eigen::Vector3d drag vector</returns>
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

/// <summary>
/// Updates the overall acceleration vector
/// </summary>
/// <param name="mass">mass of craft</param>
/// <returnsEigen::Vector3d >Acceleration Vector</returns>
Eigen::Vector3d updateAccelerationVector() {
    updateMass(mass);
    Eigen::Vector3d gravitation = updateGravitationVector();
    Eigen::Vector3d thrust = updateThrustVector(mass);
    Eigen::Vector3d drag = updateDragVector(mass);

    //cout << "G force: " << (thrust + drag).norm() / 9.81 << endl;

    return gravitation + thrust + drag;
}

/// <summary>
/// Performs an Euler numerical step
/// </summary>
void Euler() {
    static double mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY;
    Eigen::Vector3d acceleration = updateAccelerationVector();
    position = position + velocity * delta_t;
    velocity = velocity + acceleration * delta_t;
}

/// <summary>
/// Performs a Verlet numerical step
/// </summary>
void Verlet() {
    static double mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY;
    static std::vector<Eigen::Vector3d> previousStates(2);
    Eigen::Vector3d acceleration = updateAccelerationVector();
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

/// <summary>
/// Finds the rotation matrix associated with the euler angle input to the function.
/// </summary>
/// <param name="pitch">Angle of Pitch (radians)</param>
/// <param name="yaw">Angle of Yaw (radians)</param>
/// <param name="roll">Angle of Roll (radians)</param>
/// <param name="axes">Axes which define pitch, roll and yaw</param>
/// <returns></returns>
Eigen::Matrix4d quaternionRotationMatrix(double pitch, double yaw, double roll, std::vector<Eigen::Vector3d> axes) {
    
    ///Check UnitZ here and UnitZ graphical are the same by showing both. 
    Eigen::AngleAxisd rollAngle(roll, axes[2]);
    Eigen::AngleAxisd yawAngle(yaw, axes[1]);
    Eigen::AngleAxisd pitchAngle(pitch, axes[0]);

    Eigen::Quaternion<double> q = pitchAngle * yawAngle * rollAngle;
    q.normalize();

    //Update net rotation matrix
    rotQuat = q * rotQuat;
    rotQuat.normalize();

    Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity(4, 4);
    rotation.block<3, 3>(0, 0) = rotQuat.toRotationMatrix();
    orientation = rotQuat.toRotationMatrix().eulerAngles(0,1,2);


    return rotation;
}

/// <summary>
/// Adjusts the orientation of the spacecraft based upon current angular velocities
/// </summary>
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
        rotMatrix4x4 = quaternionRotationMatrix(orientation[2], orientation[1], orientation[0], axes);
        Initialised = true;
    }
    else {
        //Update timestep rotations
        double rollAngle = 0;
        double pitchAngle = angularPitchVelocity * delta_t;
        double yawAngle = angularYawVelocity * delta_t;

        //Find the new rotation matrix which defines the orientation of the craft.
        rotMatrix4x4 = quaternionRotationMatrix(pitchAngle, yawAngle, rollAngle, axes);

        //Save the rotaiton matrix to the rotationArray, which is used by openGL for visualisation
    }
    Eigen::Map<Eigen::Matrix4d>(rotationArray, 0, 0) = rotMatrix4x4;

}
/// <summary>
/// Determines if it is safe to deploy the parachute, when using the parachute autopilot
/// </summary>
/// <param name="altitude">Current Altitude</param>
/// <returns>bool safe</returns>
bool parachuteSafeToDeploy(double altitude) 
{
    double atmosphericDensity = atmospheric_density(position);
    double drag = ((0.5) * atmosphericDensity * DRAG_COEF_CHUTE * NUM_CHUTES * parachuteArea * velocity.squaredNorm() * velocity.normalized()).norm();

    if ((drag < MAX_PARACHUTE_DRAG) && (velocity.norm() < MAX_PARACHUTE_SPEED) && (altitude < 20000))
    {
        return true;
    }
    else {
        return false;
    }
}

/// <summary>
/// Determines the optimal velocity at any altitude above the surface of the planet
/// </summary>
/// <param name="altitude">altitude</param>
/// <returns>double velocity</returns>
double desiredVelocity(double altitude) 
{
    if (altitude < 2000) {
        return -(0.5 + 2.6 * Kh * altitude);
    }
    else if (altitude < 15000) {
        return -(Kh * altitude);
    }
    else if ((altitude > 50000)) {
        return -0.38 * Kh * altitude;
    }
    else {
        return -(Kh * altitude);
    }
}

/// <summary>
/// Determines the appropriate throttle for the autopliot
/// </summary>
/// <returns>double throttle</returns>
double findAutopilotThrottle()
{
    double altitude = position.norm() - MARS_RADIUS;
    double error = desiredVelocity(altitude) - velocity.dot(position.normalized());

    std::fstream outputForVisualisation;
    outputForVisualisation.open("output.txt", std::ios_base::app);
    outputForVisualisation << altitude << endl;
    outputForVisualisation << -desiredVelocity(altitude) << endl;
    outputForVisualisation << velocity.norm() << endl;
    outputForVisualisation.close();

    double Power = Kp * error;

    delta =   mass * (updateGravitationVector().norm() - updateDragVector(mass).norm()) / MAX_THRUST;

    if (parachuteSafeToDeploy(altitude) && useParachuteInAutopilot)
    {
        parachute_status = DEPLOYED;
    }

    /*cout << "altitude: " << altitude << endl;
    cout << "Desired Velocity: " << desiredVelocity(altitude) << endl;
    cout << "Error: " << error << endl;
    cout << "Power: " << Power << endl;
    cout << "Delta " << delta << endl;*/
    if (Power < -delta) {
       // cout << 0.0 << endl << endl << endl;
        return 0.0;
    }
    else if (Power > 1.0 - delta) {
       // cout << 1.0 << endl << endl << endl;
        return 1.0;
    }
    else {
       // cout << delta+Power << endl << endl << endl;
        return delta + Power;
    }

}


/// <summary>
/// Determines the appropriate throttle for the autopliot
/// </summary>
/// <returns>double throttle</returns>
double findAutopilotThrottleExamplePapers()
{
    double altitude = position.norm() - MARS_RADIUS;
    double targetAltitude = 500.0;
    double speed = velocity.norm();
    float Fequilibrium = GRAVITY * MARS_MASS * mass / pow((MARS_RADIUS + targetAltitude), 2);

    float error = altitude - targetAltitude;
    return Fequilibrium / MAX_THRUST;

}


/// <summary>
/// Logs Telemetry Data from the spacecraft
/// </summary>
/// <param name="altitude">current altitude</param>
void logTelemetry(double altitude) {
   
    if (logTelemetryData)
    {
        static double initialSimulationTime = 0;

        if (!telemetryInitialised)
        {
            //Wipe the txt output file
            std::ofstream velocityInitial = std::ofstream("Telemetry/velocity.txt", std::ios_base::out);
            velocityInitial.close();

            std::ofstream simulationTimeInitial = std::ofstream("Telemetry/simtime.txt", std::ios_base::out);
            simulationTimeInitial.close();

            std::ofstream altitudeInitial = std::ofstream("Telemetry/altitude.txt", std::ios_base::out);
            altitudeInitial.close();

            std::ofstream targetVelInitial = std::ofstream("Telemetry/targetVel.txt", std::ios_base::out);
            targetVelInitial.close();

            std::ofstream thrustInitial = std::ofstream("Telemetry/thrust.txt", std::ios_base::out);
            thrustInitial.close();

            std::ofstream fuelInitial = std::ofstream("Telemetry/fuel.txt", std::ios_base::out);
            fuelInitial.close();

            std::ofstream gforceInitial = std::ofstream("Telemetry/gforce.txt", std::ios_base::out);
            gforceInitial.close();

            std::ofstream massInitial = std::ofstream("Telemetry/mass.txt", std::ios_base::out);
            massInitial.close();

            telemetryInitialised = true;

            initialSimulationTime = simulation_time;

        }

        std::ofstream velocityOut = std::ofstream("Telemetry/velocity.txt", std::ios_base::app);
        std::ofstream simulationTimeOut = std::ofstream("Telemetry/simtime.txt", std::ios_base::app);
        std::ofstream altitudeOut = std::ofstream("Telemetry/altitude.txt", std::ios_base::app);
        std::ofstream targetVelOut = std::ofstream("Telemetry/targetVel.txt", std::ios_base::app);
        std::ofstream thrustOut = std::ofstream("Telemetry/thrust.txt", std::ios_base::app);
        std::ofstream fuelOut = std::ofstream("Telemetry/fuel.txt", std::ios_base::app);
        std::ofstream gforceOut = std::ofstream("Telemetry/gforce.txt", std::ios_base::app);
        std::ofstream massOut = std::ofstream("Telemetry/mass.txt", std::ios_base::app);

        velocityOut << velocity.norm() << endl;
        simulationTimeOut << simulation_time - initialSimulationTime << endl;
        altitudeOut << altitude << endl;
        targetVelOut << -desiredVelocity(altitude) << endl;
        thrustOut << thrust_wrt_world().norm() << endl;
        fuelOut << fuel << endl;
        gforceOut << (updateThrustVector(mass) + updateDragVector(mass)).norm() / 9.81 << endl;
        massOut << mass << endl;

        velocityOut.close();
        simulationTimeOut.close();
        altitudeOut.close();
        targetVelOut.close();
        thrustOut.close();
        fuelOut.close();
        gforceOut.close();
        massOut.close();
    }

}