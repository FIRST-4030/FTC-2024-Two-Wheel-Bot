/*
Java class that keeps track of a two-wheeled balancing robot's position using encoder values from the wheels.
Origial code from ChatGPT
The class converts the encoder values into the robot's X and Y position and rotation (theta or yaw).
Added method of subtracting the robot pitch from position.
Robot pitch may move the encoders but not the change robot position
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class Odometry {
    // Robot parameters
    private ElapsedTime runtime = new ElapsedTime();
    private double wheelBase; // distance between the wheels
    private double wheelCircumference;  // wheel circumference
    
    private double lastLeftDistance = 0;
    private double lastRightDistance = 0;
    private double lastPitch = 0;

    // Position and orientation
    private double x = 0;
    private double y = 0;
    private double theta = 0; // orientation angle in radians
    private double noNormalTheta = 0;
    private double deltaDistance = 0.0; 
    private double deltaTheta = 0.0;  

    private double linearVelocity = 0.0;
    private RunningAverage veloAvg;   // USES THE RunningAverage class
    
    private double lastTime;  // last timestamp;

    // Constructor,  provide wheel base in mm
    public Odometry(double wheelBase, double wheelDia, double initialPitch) {
        this.wheelBase = wheelBase;
        this.wheelCircumference = wheelDia*Math.PI; // convert diameter to circumference
        this.lastPitch = initialPitch; // robots initial pitch, probably not zero
        this.lastTime = runtime.seconds();
        this.veloAvg = new RunningAverage(4);
    }

    /**
     * Updates the position and orientation of the robot based on new encoder values.
     *
     * @param leftDistance  The new distance traveled by the left wheel (in mm).
     * @param rightDistance The new distance traveled by the right wheel (in mm).
     */
    public void update(double leftDistance, double rightDistance, double pitch) {
        
        // Calculate the delta pitch (degrees) of the chassis, since the last update
        // This needs to be subtracted from the travel
        // Trying to subtract out the balancing motion from the robots position
        double deltaPitch = pitch - lastPitch;
        lastPitch = pitch;
        double pitchEqDist = (deltaPitch/360.0)*wheelCircumference; // Pitch Equivalent Distance
        
        // Calculate the distance traveled by each wheel since the last update
        double deltaLeft  = leftDistance -lastLeftDistance; //- pitchEqDist;
        double deltaRight = rightDistance-lastRightDistance; // - pitchEqDist;

        // Update the last distances
        lastLeftDistance = leftDistance;
        lastRightDistance = rightDistance;

        // Calculate delta time
        double now = runtime.seconds();
        double timeChange = (now-lastTime); 
        lastTime = now;
        
        // Calculate the change in orientation
        deltaTheta = (deltaLeft- deltaRight) / wheelBase;

        // Calculate the average distance traveled
        deltaDistance = (deltaLeft + deltaRight) / 2;
        linearVelocity = deltaDistance/timeChange;
        veloAvg.addNumber(linearVelocity); // add to the running average

        // Update the position and orientation
        if (deltaTheta == 0) {
            // Robot is moving straight
            x += deltaDistance * Math.cos(theta);
            y += deltaDistance * Math.sin(theta);
        } else {
            // Robot is rotating around a point
            double radius = deltaDistance / deltaTheta;
            double centerX = x - radius * Math.sin(theta);
            double centerY = y + radius * Math.cos(theta);
            theta += deltaTheta;
            x = centerX + radius * Math.sin(theta);
            y = centerY - radius * Math.cos(theta);
        }

        // Normalize theta to the range [-pi, pi]
        noNormalTheta = theta;
        theta = normalizeAngle(theta);
    }

    /**
     * Normalizes an angle to the range [-pi, pi].
     *
     * @param angle The angle to normalize (in radians).
     * @return The normalized angle (in radians).
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // Getters for position and orientation
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return noNormalTheta;
    }

    public double getLinearVelocity() {
        return linearVelocity; // return instant linear velocity
    }
    public double getAvgLinearVelocity() {
        return veloAvg.getAverage(); // return average linear velocity
    }
}

