package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Two Wheel Balancing Robot Arm Object, driven by a servo.
 * Computes Arm Servo value and Pitch setpoint, for desired arm angle.
 * This keeps the Center of Gravity (CG) of Arm + Robot Body over the Robot Wheel axis.
 * The computation has been done externally (using OpenSCAD Mass Properties Simulator program)
 * and saved as a lookup Piecewise curve.
  */
public class TWBArmServo {
    private double currentPos; // current position (degrees)
    private double targetPos; // target position (degrees)
    final private double posMax; // maximum position to prevent damage (degrees)
    final private double posMin; // minimum position to prevent damage (degrees)
    final private double maxVelocity; // maximum deg/sec allowed (always positive, work either direction)

    final private Servo armServo;

    // CONSTANTS TO CONVERT LOCAL ARM ANGLES to SERVO VALUES
    // Using a REV 5 turn servo with a 2:1 gear ratio
    // Total possible arm rotation is 2.5 turns * 360 deg = 900 degrees
    // Actual ARMROTDEG varies based on measurement (MIGHT BE 750)
    static final double ARMROTDEG = 800.0; // Max degrees arm can rotate, unconstrained

    // PieceWise linear curve member for pitch angle vs arm angle
    final private PiecewiseFunction pitchAngVec = new PiecewiseFunction();

    /**
     * TWBArmServo constructor. Initializes the pitch vs arm-angle curve.
      */
    public TWBArmServo(HardwareMap hardwareMap, String servoName, double initAngle,
                       double maxPos, double minPos, double maxV){
        this.currentPos = initAngle;
        this.targetPos = initAngle;
        this.posMax = maxPos;
        this.posMin = minPos;
        this.maxVelocity = maxV;

        armServo = hardwareMap.get(Servo.class, servoName);

        pitchAngVec.debug = false;
        pitchAngVec.setClampLimits(false);
        pitchAngVec.addElement(-160,4.26974); // new global arm angle is -155.73
        pitchAngVec.addElement(-140,7.69909); // new global arm angle is -132.301
        pitchAngVec.addElement(-120,9.78656); // new global arm angle is -110.213
        pitchAngVec.addElement(-100,10.4355); // new global arm angle is -89.5645
        pitchAngVec.addElement(-80,9.81183); // new global arm angle is -70.1882
        pitchAngVec.addElement(-60,8.18526); // new global arm angle is -51.8147
        pitchAngVec.addElement(-40,5.83785); // new global arm angle is -34.1622
        pitchAngVec.addElement(-20,3.03044); // new global arm angle is -16.9696
        pitchAngVec.addElement(0,0); // new global arm angle is 0
        pitchAngVec.addElement(20,-3.03044); // new global arm angle is 16.9696
        pitchAngVec.addElement(40,-5.83785); // new global arm angle is 34.1622
        pitchAngVec.addElement(60,-8.18526); // new global arm angle is 51.8147
        pitchAngVec.addElement(80,-9.81183); // new global arm angle is 70.1882
        pitchAngVec.addElement(100,-10.4355); // new global arm angle is 89.5645
        pitchAngVec.addElement(120,-9.78656); // new global arm angle is 110.213
        pitchAngVec.addElement(140,-7.69909); // new global arm angle is 132.301
        pitchAngVec.addElement(160,-4.26974); // new global arm angle is 155.73
        }

    /**
     * TWBArmServo method.  Updates the target arm position.
     * @param armAngle
     */
    public void setArmAngle(double armAngle) {targetPos = armAngle;}

    /**
     * TWBArmServo method.  Call this continuously to move the arm.
     * @param deltaTime
     * @return target pitch angle for robot
     */
    public double updateArm(double deltaTime){
        // Update the arm smoothly
        updatePosition(deltaTime);

        // convert the arm angle into a servo value from 0 to 1
        double servoTarget = ((-currentPos+(ARMROTDEG/2.0))/ARMROTDEG);

        armServo.setPosition(servoTarget); // this moves the servo

        // return the robot pitch to match the desired arm angle, based on balanced cg
        return this.getPitchAngle(currentPos);
    }

    /**
     * TWBArmServo method.  Call this continuously to provide smooth arm position update.
     * @param deltaTime
     */
    private void updatePosition(double deltaTime) {
        double deltaPos = targetPos-currentPos; // position change being asked for
        double deltaPosMax = maxVelocity * deltaTime; // convert rate limit to delta position based on loop time
        // apply velocity (rate) limit
        if (Math.abs(deltaPos) > deltaPosMax) deltaPos = Math.signum(deltaPos) * deltaPosMax;

        currentPos += deltaPos; // update current position

        // check limits
        if (currentPos > posMax) targetPos = posMax;
        else if (currentPos < posMin) targetPos = posMin;
    }

    /**
     * return the required body pitch angle given the desired arm angle
      */
    private double getPitchAngle(double armDesiredAngle){
        return pitchAngVec.getY(armDesiredAngle);
    }

    /**
     * TWBArmServo method.  Returns the current position of the arm.
     * @return currentPos
     */
    public double getAngle() { return currentPos;}

    /**
     * TWBArmServo method. Returns the target position of the arm.
     * @return targetPos
     */
    public double getTargetAngle() { return targetPos; }
}
