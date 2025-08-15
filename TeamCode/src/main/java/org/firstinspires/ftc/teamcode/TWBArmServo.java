package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Special Two Wheel Balancing Robot Arm Object, driven by a servo.
// Computes Arm Servo value and Pitch setpoint, for desired arm angle.
// This keeps the Center of Gravity (CG) of Arm + Robot Body over the Robot Wheel axis.
// The Computation has been done externally (OpenSCAD Mass Properties Simulator program)
// and saved as lookup Piecewise curve.
public class TWBArmServo {
    private TWBMotionControl armAngleObj; // object to hold arm angle
    private Servo armServo;

    // CONSTANTS TO CONVERT LOCAL ARM ANGLES to SERVO VALUES
    // Using a REV 5 turn servo with a 2:1 gear ratio
    // Total possible arm rotation is 2.5 turns * 360 deg = 900 degrees
    // Actual ARMROTDEG varies based on measurement
    static final double ARMROTDEG = 800.0; // Max degrees arm can rotate, unconstrained

    // PieceWise linear curve member for pitch angle vs arm angle
    final private PiecewiseFunction pitchAngVec = new PiecewiseFunction();

    // constructor
    public TWBArmServo(HardwareMap hardwareMap, String servoName, double initAngle){
        armAngleObj = new TWBMotionControl(initAngle, 120.0, -160.0, 80.0,
                0.0, 2000.0);

        armServo = hardwareMap.get(Servo.class, servoName);

        pitchAngVec.debug = false;
        pitchAngVec.setClampLimits(false);
        pitchAngVec.addElement(-140,9.48606); // new global arm angle is -130.514
        pitchAngVec.addElement(-120,11.8906); // new global arm angle is -108.109
        pitchAngVec.addElement(-100,12.5079); // new global arm angle is -87.4921
        pitchAngVec.addElement(-80,11.626); // new global arm angle is -68.374
        pitchAngVec.addElement(-60,9.61346); // new global arm angle is -50.3865
        pitchAngVec.addElement(-40,6.81452); // new global arm angle is -33.1855
        pitchAngVec.addElement(-20,3.52474); // new global arm angle is -16.4753
        pitchAngVec.addElement(0,0); // new global arm angle is 0
        pitchAngVec.addElement(20,-3.52474); // new global arm angle is 16.4753
        pitchAngVec.addElement(40,-6.81452); // new global arm angle is 33.1855
        pitchAngVec.addElement(60,-9.61346); // new global arm angle is 50.3865
        pitchAngVec.addElement(80,-11.626); // new global arm angle is 68.374
        pitchAngVec.addElement(100,-12.5079); // new global arm angle is 87.4921
        pitchAngVec.addElement(120,-11.8906); // new global arm angle is 108.109
        pitchAngVec.addElement(140,-9.48606); // new global arm angle is 130.514
        }

    public void setArmAngle(double armAngle) {armAngleObj.setTargetPos(armAngle);}

    public double updateArm(double deltaTime){
        // TAKE DESIRED ARM ANGLE AND SET SERVO VALUE AND RETURN PITCH SETPOINT

        armAngleObj.updatePosition(deltaTime, true);

        // convert the arm angle into a servo value from 0 to 1
        double servoTarget = ((-armAngleObj.getCurrentPos()+(ARMROTDEG/2.0))/ARMROTDEG);

        armServo.setPosition(servoTarget);

        // return the robot pitch to match the desired arm angle, based on balanced cg
        return this.getPitchAngle(armAngleObj.getCurrentPos());
    }

    // return the required body pitch angle given the desired arm angle
    public double getPitchAngle(double armDesiredAngle){
        return pitchAngVec.getY(armDesiredAngle);
    }
    // return the servo position (0 to 1).  This returns the last command sent to the servo
    public double getPosition() {
        return armServo.getPosition();
    }

    public double getAngle() { return armAngleObj.getCurrentPos();}

    public double getTargetAngle() { return armAngleObj.getTargetPos(); }
}
