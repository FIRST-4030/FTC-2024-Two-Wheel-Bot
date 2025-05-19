package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ServoHoldPosition {
    //Servo bounds 0.38-0.62
    public Servo armServo;
    private final IMU imu;
    private double pitchError;
    //Conversion from degrees provided by the IMU to modification of 0.00-1.00 Servo, accounting for gearing and limits
    private final double DEGREES_TO_TRGT_POS = (double)1/900;

    public ServoHoldPosition(HardwareMap hardwareMap, String servoName, IMU imu){
        armServo = hardwareMap.get(Servo.class, servoName);
        this.imu = imu;
        update(0.48);
    }

    //Checks the IMU's provided pitch and changes the servo target position to account for the pitch.
    public void update(double servoTarget){
        pitchError = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        if(pitchError < 15) {
            servoTarget -= pitchError * DEGREES_TO_TRGT_POS;
        }
        if(servoTarget < 0.27) servoTarget = 0.27;
        if(servoTarget > 0.70) servoTarget = 0.70;
        armServo.setPosition(servoTarget);
    }
}
