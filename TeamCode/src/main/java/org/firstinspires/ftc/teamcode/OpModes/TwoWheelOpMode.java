package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drives.TwoWheelDrive;
import org.firstinspires.ftc.teamcode.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;

@TeleOp
public class TwoWheelOpMode extends OpMode {
    TwoWheelDrive drive;
    InputHandler inputHandler;
    Vector2d control;
    ElapsedTime runTimer;
    @Override
    public void init() {
        runTimer = new ElapsedTime();
        runTimer.reset();
        drive = new TwoWheelDrive(hardwareMap);
        inputHandler = InputAutoMapper.normal.autoMap(this);
    }

    @Override
    public void loop() {
        handleInput();
        outputLog(drive);
        drive.update(control);
    }

    public void handleInput(){
        inputHandler.loop();
        control.y = gamepad1.right_stick_y;
        control.x = gamepad1.left_stick_x;
    }
    public void outputLog(TwoWheelDrive drive){
        RobotLog.d("WAY: Current Robot IMU values: Angular Velocity: %.03f Yaw, Pitch, Roll: %.03f Runtime: %.03f", drive.getImu().getRobotAngularVelocity(AngleUnit.RADIANS), drive.getImu().getRobotYawPitchRollAngles(), runTimer.milliseconds());
    }
}
