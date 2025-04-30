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

    public ServoHoldPosition(HardwareMap hardwareMap, String servoName, IMU imu){
        armServo = hardwareMap.get(Servo.class, servoName);
        this.imu = imu;
        update(0.48);
    }

    public void update(double servoTarget){
        pitchError = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        servoTarget -= pitchError * 1/900;
        if(servoTarget < 0.36) servoTarget = 0.36;
        if(servoTarget > 0.60) servoTarget = 0.60;
        armServo.setPosition(servoTarget);
    }
}
