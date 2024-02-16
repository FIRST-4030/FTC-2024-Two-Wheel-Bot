package org.firstinspires.ftc.teamcode.Drives;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.core.maths.vectors.Vector2d;

import java.util.Vector;

public class TwoWheelDrive {
    DcMotorEx leftWheelMotor;
    DcMotorEx rightWheelMotor;
    IMU imu;
    public TwoWheelDrive(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu");
        leftWheelMotor = hardwareMap.get(DcMotorEx.class, "LeftMotor");
        rightWheelMotor = hardwareMap.get(DcMotorEx.class, "RightMotor");
        leftWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update(Vector2d control){
        double joystickY = -1* control.y;
        double joystickX = control.x;

        double normalization = Math.max(Math.abs(joystickX) + Math.abs(joystickY), 1);
        leftWheelMotor.setPower((joystickY + joystickX)/normalization);
        rightWheelMotor.setPower((joystickY - joystickX)/normalization);

    }

    public IMU getImu(){
        return imu;
    }
}
