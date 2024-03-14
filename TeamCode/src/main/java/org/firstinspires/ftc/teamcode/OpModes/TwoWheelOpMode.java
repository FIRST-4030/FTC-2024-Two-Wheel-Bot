package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Drives.TwoWheelDrive;
import org.firstinspires.ftc.teamcode.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;

@TeleOp(name = "Two Wheel Drive")
public class TwoWheelOpMode extends OpMode {
    TwoWheelDrive drive;
    InputHandler inputHandler;
    YawPitchRollAngles orientation;
    double currentOffset = 0;
    double p = 1.75;
    double d = 0.001;
    double currentAccelerationX = 0;
    double currentAccelerationY = 0;
    double currentAccelerationZ = 0;

    Vector2d control;
    ElapsedTime runTimer;
    double pitchVelocity = 0;
    @Override
    public void init() {
        runTimer = new ElapsedTime();
        runTimer.reset();
        control = new Vector2d();
        drive = new TwoWheelDrive(hardwareMap);
        inputHandler = InputAutoMapper.normal.autoMap(this);

    }

    @Override
    public void loop() {
        handleInput();
        orientation = drive.getImu().getRobotYawPitchRollAngles();
        /*currentAccelerationX = drive.getAccelerometer().getAcceleration().xAccel;
        currentAccelerationY = drive.getAccelerometer().getAcceleration().yAccel;
        currentAccelerationZ = drive.getAccelerometer().getAcceleration().zAccel;*/
        currentOffset = orientation.getPitch(AngleUnit.RADIANS);
        AngularVelocity angularVelocity = drive.getImu().getRobotAngularVelocity(AngleUnit.DEGREES);
        pitchVelocity = angularVelocity.xRotationRate;
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
        telemetry.addData("Current p:", p);
        telemetry.addData("Current p:", d);
        telemetry.addData("Motor Power Y: ", control.y);
        telemetry.addData("Motor Power X:", control.x);
        telemetry.update();
        /*telemetry.addData("X Accel: ", currentAccelerationX);
        telemetry.addData("Y Accel: ", currentAccelerationY);
        telemetry.addData("Z Accel: ", currentAccelerationZ);*/


        outputLog(drive);
        drive.update(control);
    }

    public void handleInput(){
        inputHandler.loop();
        control.y = (currentOffset * p) - (pitchVelocity * d);
        if(inputHandler.up("D1:DPAD_UP")){
            p += 0.01;
        }
        if(inputHandler.up("D1:DPAD_DOWN")){
            p -= 0.01;
        }
        if(inputHandler.up("D1:DPAD_LEFT")){
            d -= 0.001;
        }
        if(inputHandler.up("D1:DPAD_RIGHT")){
            d += 0.001;
        }
        //control.y = gamepad1.right_stick_y;
        //control.x = gamepad1.right_stick_x;
    }
    public void outputLog(TwoWheelDrive drive){

    }
}
