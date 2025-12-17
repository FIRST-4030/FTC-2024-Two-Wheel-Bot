package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This OpMode runs a flywheel shooter motor using a custom feedforward + feedback controller.
 * It also does Datalogging, to review the performance of the shooter.
 */
@Disabled
@TeleOp(name="ShooterController")
public class ShooterController extends LinearOpMode {

    // our DC motor
    DcMotorEx shooter;

    // Feedforward term is Kvelo (velocity term)
    //    Tune the Kvelo term first.
    public static final double Kvelo = 0.0243; // power multiplier for rotations per second
    // FeedBack term is Kp (proportional term)
    //   Set Kp to zero when tuning the Kvelo term!!
    public static final double Kp = 0.3;  // no gain in improvement when increasing beyond this

    static final double   COUNTS_PER_REV = 28 ;  // REV HD Hex 1:1 Motor Encoder

    private double targetVelocity = 10;  // rotations per second (max is ~40)

    Datalog datalog; // create the data logger object

    private int i = 0; // loop counter

    // Timer
    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        // Get reference to DC motor.
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // WITH OUT!

        // Initialize the datalog
        datalog = new Datalog("ShooterLog10_19");

        // wait for start command
        waitForStart();

        runtime.reset(); // reset the clock

        // display info to user
        while(opModeIsActive()) {

            i++;

            if (gamepad1.xWasPressed()) targetVelocity += 5; // closed value (0.98 for blocks)
            if (gamepad1.bWasPressed()) targetVelocity -= 5; // open value (WAS 0.35)

            double currentVelocity = shooter.getVelocity(AngleUnit.DEGREES)/COUNTS_PER_REV;

            double veloError = targetVelocity - currentVelocity;

            // CONTROLLER:  feedfoward = Kvelo + feedback = Kpos
            double setPower = targetVelocity * Kvelo  + veloError * Kp;

            shooter.setPower(setPower);

            telemetry.addData("shooter Velocity Target X and B", targetVelocity);
            telemetry.addData("shooter Velocity Actual", currentVelocity);
            telemetry.addData("setPower value", setPower);

            telemetry.update();

            // Data log
            // Note that the order in which we set datalog fields
            // does *not* matter! Order is configured inside the Datalog class constructor.
            datalog.loopCounter.set(i);
            datalog.runTime.set(runtime.seconds());
            datalog.setPower.set(setPower);
            datalog.posError.set(veloError);
            datalog.shooterVelocity.set(currentVelocity);
            datalog.targetVelocity.set(targetVelocity);
            datalog.writeLine();
        }
    }

/**
 * Datalog class encapsulates all the fields that will go into the datalog.
 */
public static class Datalog {
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField loopCounter = new Datalogger.GenericField("LoopCounter");
    public Datalogger.GenericField runTime = new Datalogger.GenericField("RunTime");
    public Datalogger.GenericField setPower = new Datalogger.GenericField("setPower");
    public Datalogger.GenericField posError = new Datalogger.GenericField("posError");

    public Datalogger.GenericField shooterVelocity = new Datalogger.GenericField("shooterVelocity");
    public Datalogger.GenericField targetVelocity = new Datalogger.GenericField("targetVelocity");

    public Datalog(String name) {
        // Build the underlying datalog object
        datalogger = new Datalogger.Builder()

                // Pass through the filename
                .setFilename(name)

                // Request an automatic timestamp field
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                // Tell it about the fields we care to log.
                // Note that order *IS* important here! The order in which we list
                // the fields is the order in which they will appear in the log.
                .setFields(
                        loopCounter,
                        runTime,
                        setPower,
                        posError,
                        shooterVelocity,
                        targetVelocity
                )
                .build();
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine() {
        datalogger.writeLine();
    }
}
}