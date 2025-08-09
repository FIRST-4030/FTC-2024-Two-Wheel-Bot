package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Special Two Wheel Balancing Robot Arm Object.
// Computes Arm Servo value and Pitch setpoint, for desired absolute arm angle, to keep
// the Center of Gravity (CG) of Arm + Robot Body over the Robot Wheel axis.
// The Computation has been done externally (OpenSCAD Mass Properties Simulator program)
// and saved as lookup Piecewise curves.
public class ServoHoldPosition {
    public Servo armServo; // WHY IS THIS MEMBER PUBLIC?

    // CONSTANTS TO CONVERT LOCAL ARM ANGLES to SERVO VALUES
    // Using a REV 5 turn servo with a 2:1 gear ratio
    // Total possible arm rotation is 2.5 turns * 360 deg = 900 degrees
    static final double ARMROTDEG = 900.0; // Max degrees arm can rotate, unconstrained

    // Constrain Arm angle so that robot does not break self
    static final double MAXARM = 140.0; // Max degrees constrained

    // PieceWise linear curve member for servo angle vs desired arm angle
    final private PiecewiseFunction armAngVec = new PiecewiseFunction();

    // PieceWise linear curve member for pitch angle vs desired arm angle
    final private PiecewiseFunction pitchAngVec = new PiecewiseFunction();

    // constructor
    public ServoHoldPosition(HardwareMap hardwareMap, String servoName){
        armServo = hardwareMap.get(Servo.class, servoName);
        //update(0.0);

        armAngVec.debug = false;
        armAngVec.addElement(-120,-141.107);
        armAngVec.addElement(-100,-124.174);
        armAngVec.addElement(-80,-104.174);
        armAngVec.addElement(-60,-81.1073);
        armAngVec.addElement(-40,-55.503);
        armAngVec.addElement(-20,-28.1764);
        armAngVec.addElement(0,0);
        armAngVec.addElement(20,28.1764);
        armAngVec.addElement(40,55.503);
        armAngVec.addElement(60,81.1073);
        armAngVec.addElement(80,104.174);
        armAngVec.addElement(100,124.174);
        armAngVec.addElement(120,141.107);

        pitchAngVec.debug = false;
        pitchAngVec.addElement(-120,21.1073);
        pitchAngVec.addElement(-100,24.174);
        pitchAngVec.addElement(-80,24.174);
        pitchAngVec.addElement(-60,21.1073);
        pitchAngVec.addElement(-40,15.503);
        pitchAngVec.addElement(-20,8.17639);
        pitchAngVec.addElement(0,0);
        pitchAngVec.addElement(20,-8.17639);
        pitchAngVec.addElement(40,-15.503);
        pitchAngVec.addElement(60,-21.1073);
        pitchAngVec.addElement(80,-24.174);
        pitchAngVec.addElement(100,-24.174);
    }

    public double update(double armDesiredAngle){
        // TAKE DESIRED ARM ANGLE AND SET SERVO VALUE AND RETURN PITCH SETPOINT
        double armAngle = this.getArmAngle(armDesiredAngle);

        // constrain the arm angle
        if (armAngle > MAXARM) armAngle = MAXARM;
        else if (armAngle < -1*MAXARM) armAngle = -1*MAXARM;

        // convert the arm angle into a servo value from 0 to 1
        double servoTarget = ((armAngle+450)/ARMROTDEG);

        armServo.setPosition(servoTarget);

        // return the robot pitch to match the desired arm angle, based on balanced cg
        return this.getPitchAngle(armDesiredAngle);
    }

    // return the required arm angle given the desired arm angle
    public double getArmAngle(double armDesiredAngle){
        return armAngVec.getY(armDesiredAngle);
    }

    // return the required body pitch angle given the desired arm angle
    public double getPitchAngle(double armDesiredAngle){
        return pitchAngVec.getY(armDesiredAngle);
    }
    // return the servo position (0 to 1).  This returns the last command sent to the servo
    public double getPosition() {
        return armServo.getPosition();
    }
}
