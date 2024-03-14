package org.firstinspires.ftc.teamcode.Drives;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.core.maths.vectors.Vector2d;

import java.util.Vector;

public class TwoWheelDrive {
    DcMotorEx leftWheelMotor;
    DcMotorEx rightWheelMotor;
    IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    // Now initialize the IMU with this mounting orientation
    // Note: if you choose two conflicting directions, this initialization will cause a code exception.
    AndroidAccelerometer accelerometer;
    public TwoWheelDrive(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        accelerometer = new AndroidAccelerometer();
        accelerometer.startListening();
        leftWheelMotor = hardwareMap.get(DcMotorEx.class, "LeftMotor");
        rightWheelMotor = hardwareMap.get(DcMotorEx.class, "RightMotor");
        leftWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        leftWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update(Vector2d control){
        double joystickY = -1 * control.y;
        double joystickX = control.x;

        double normalization = Math.max(Math.abs(joystickX) + Math.abs(joystickY), 1);
        leftWheelMotor.setPower(((joystickY + joystickX)/normalization));
        rightWheelMotor.setPower(((joystickY - joystickX)/normalization));


    }

    public IMU getImu(){
        return imu;
    }

    public AndroidAccelerometer getAccelerometer() {return accelerometer;}
}
