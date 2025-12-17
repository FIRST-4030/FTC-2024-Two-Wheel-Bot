/*
This class is for a Two Wheel Balancing Robot With Arm
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Two Wheel Balancing Robot Class, with Arm.
 * This class has about 50 members (variables)!
 */
public class TwoWheelBalanceBot {
    final private OpMode theOpmode; // Set during construction.  Enables using telemetry and gamepad
    public boolean LOG = true;  // should the log be recorded?

    public boolean TELEMETRY = true; // switch for telementry

    public boolean APRILTAG = false; // switch for AprilTags

    Datalog datalog; // create the data logger object

    // These are the state terms for a two wheel balancing robot
    public double Kpitch = -0.61; // volts/degree at arm = -90
    public double KpitchRate = -0.022; // volts/degrees/sec
    // Have had difficulty tuning this term.  Can't tell what changes it makes.

    public double Kpos = 0.009;  // volts/mm For high balancing (unstable) this term is positive
    public double Kvelo = 0.015;  // volts/mm/sec For high balancing (unstable) this term is positive
    // Larger Kvelo decreases the rocking motion, up to a point, then chatter!
    // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles.

    static final double TICKSPERMM = 1.7545; // REV SPUR 40:1, 8in wheels
    static final double WHEELBASE = 300; // robot Wheel base (mm)
    static final double WHEELDIA = 203.0; // 8 inch wheel diameter (mm)

    // YAW PID
    PIDController yawPID = new PIDController(0.45, 0.12, 0.05); // kp, ki, kd

    public double posTarget = 0.0;
    public double veloTarget = 0.0;
    public double autoPitchTarget = 0; // used to set pitch from an auto routine
    double pitchTarget = 0;

    double pitch = 0; // degrees, value got from imu
    double zeroVoltsAdjust = 0.0; //
    double pitchError = 0;

    public double yawTarget = 0.0; // from the user joystick in teleop or from auto routines
    double yaw = 0;
    double priorYaw = 0;
    double rawYaw, rawPriorYaw = 0;
    double yawRATE; // not used other than for datalog
    double yawPower;
    public boolean ClawIsClosed = false; //Claw boolean

    final private IMU imu;
    private YawPitchRollAngles orientation;   // part of FIRST navigation classes

    final private VoltageSensor battery;
    double currentVoltage = 12.0;

    //Handles the arm control, and adjusting the arm for the pitch of the robot
    final public TWBArmServo theArm;

    public DcMotor leftDrive;
    public DcMotor rightDrive;

    TWBOdometry odometry; // two wheel odometry object with running average
    int i = 0;  // loop counter, used with data logging

    // Timers
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime distTimer = new ElapsedTime();
    private final RunningAverage deltaTimeRA = new RunningAverage(8); // Running average of linear velocity
    private double currentTime;
    double deltaTime = 0.02;

    public double sOdom; // Current robot position from odometry
    double theta;  // used with odometry

    double armPitchTarget = 0; // degrees

    //Timer to limit how frequently the claw opens the closes
    ElapsedTime clawTimer = new ElapsedTime();
    public Servo clawServo;

    // used in Design of Experiments
    public double PosAmplitude = 0;
    public double PitchAmplitude = 0;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    public AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private HardwareMap hwmp;
    /**
     * TWB Constructor.  Call once in initialization.
      */
    public TwoWheelBalanceBot(HardwareMap hardwareMap, OpMode opMode) {

        this.theOpmode = opMode; // set the opmode that is calling this class
        // Initialize the datalog

        this.hwmp = hardwareMap;

        deltaTimeRA.addNumber(0.04); // add to running average to smooth start

        // Define and Initialize Motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometry = new TWBOdometry(WHEELBASE, WHEELDIA, pitch); // create odometry object

        // Get devices from the hardwareMap.
        // as needed, change "Control Hub" to (e.g.) "Expansion Hub 1".
        battery = hardwareMap.voltageSensor.get("Control Hub");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        // Initialize the arm class
        // ARM LIMITS ARE DEFINED IN ArmServoTWB class
        theArm = new TWBArmServo(hardwareMap, "arm_servo", 0.0, 140, -165, 60);
        // NOTE: Set arm angle to zero to rig the servo (so it is easy to see)

        clawServo = hardwareMap.get(Servo.class, "clawServo");

        yawPID.setSetpoint(0.0);    // initial yaw (yawTarget) is zero.

    }

    /**
     * TWB init. Called once at initialization
     */
    public void init() {
        if (LOG) datalog = new Datalog("TwoWheelBotDec15");

        if (APRILTAG) {

            // Create the AprilTag processor.
            aprilTag = new AprilTagProcessor.Builder()

                    // The following default settings are available to un-comment and edit as needed.
                    //.setDrawAxes(false)
                    //.setDrawCubeProjection(false)
                    //.setDrawTagOutline(true)
                    .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                    .setCameraPose(cameraPosition, cameraOrientation)

                    .build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // eg: Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            aprilTag.setDecimation(6);

            // Create the vision portal by using a builder.
            VisionPortal.Builder builder = new VisionPortal.Builder();

            builder.setCamera(hwmp.get(WebcamName.class, "Webcam 1"));

            builder.enableLiveView(true);
            //builder.setLiveViewContainerId(1);

            // Set and enable the processor.
            builder.addProcessor(aprilTag);

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();
        }

    }
    /**
     * TWB init loop.  Called repeatedly in initialization.
      */
    public void init_loop() {
        theOpmode.telemetry.addData("LOG", LOG);
        theOpmode.telemetry.addData("APRIL TAG", APRILTAG);
    }

    /**
     *  TWB automatic self righting method.  Call repeatedly in initialization.
      */
    public void auto_right_loop() {
        theOpmode.telemetry.addData("AUTO-RIGHT", "ACTIVE");
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        theOpmode.telemetry.addData("Pitch (DEG)", pitch);

        // Check which way the robot is leaning and rotate the arm so that it will self-right
        if (pitch > 0.0) theArm.setArmAngle(140.0);
        else theArm.setArmAngle(-150.0);
        theOpmode.telemetry.addData("Arm Angle (DEG)", theArm.getAngle());
        armPitchTarget = theArm.updateArm(0.02); // This will make the arm move
    }

    /**
     *  TWB Servo rigging method. Gets the Arm and Claw servos ready for attaching the horns (rigging)
     *  Arm should be straight up.  Claw should be closed.
     *  Call repeatedly in initialization.
      */
    public void servo_rig_loop() {
        theOpmode.telemetry.addData("ARM", "RIGGING");
        theOpmode.telemetry.addData("ATTACH ARM", "STRAIGHT UP");

        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        theOpmode.telemetry.addData("Pitch (DEG)", pitch);

        theArm.setArmAngle(0.0);
        theOpmode.telemetry.addData("Arm Angle (DEG)", theArm.getAngle());
        armPitchTarget = theArm.updateArm(0.02); // This will make the arm move

        //clawServo.setPosition(0.98);
        //theOpmode.telemetry.addData("CLAW", "RIGGING");
        //theOpmode.telemetry.addData("ATTACH CLAW", "CLOSED");
    }

    /**
     * TWB start method. Called once on Start press. Resets encoders, timers, PIDs
      */
    public void start() {
        // reset the encoders because the robot may have moved during auto righting
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        imu.resetYaw(); // set the yaw value to zero

        // reset the timers for the datalog timestamp, turns, runToPos
        runtime.reset();
        distTimer.reset();

        // reset the PIDs
        yawPID.reset();

        // Get the current voltage just before loop, so that balance control is more consistent
        // Note: getting voltage in loop() can cause feedback issues
        currentVoltage = battery.getVoltage();
        currentTime = runtime.seconds(); // initialize current time for delta time values

        theArm.setArmAngle(0.0);
    }

    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    public void loop() {
        double leftTicks = leftDrive.getCurrentPosition();
        double rightTicks = rightDrive.getCurrentPosition();
        AngularVelocity angularVelocity;  // part of FIRST navigation classes

        int at_id = 0;
        double at_range = 0;
        double at_bearing = 0;

        i++;  // index the loop counter

        // compute a loop time.  Using running average to smooth values
        double lastTime = currentTime;
        currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;
        // add the new delta time to the running average
        deltaTimeRA.addNumber(deltaTime);
        deltaTime = deltaTimeRA.getAverage(); // replace deltaTime with running average delta time

        // get values from the IMU
        orientation = imu.getRobotYawPitchRollAngles();
        //yaw = orientation.getYaw(AngleUnit.DEGREES);
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        double pitchRATE = angularVelocity.xRotationRate;
        yawRATE = angularVelocity.zRotationRate;

        // get values from wheel encoders (odometry)
        odometry.update(leftTicks / TICKSPERMM, rightTicks / TICKSPERMM, pitch, deltaTime);
        double linearVelocity = odometry.getAvgLinearVelocity();
        sOdom = odometry.getS();
        theta = odometry.getTheta(); // should be the same value as imu yaw

        // The robot pitch target is determined by the angle of the arm.
        armPitchTarget = theArm.updateArm(deltaTime); // update arm position (this makes it move)



        // MAIN BALANCE CONTROL CODE:
        double posError = sOdom - posTarget;
        double veloError = linearVelocity + veloTarget;
        double positionVolts = Kvelo * veloError + Kpos * posError;

        pitchTarget = armPitchTarget + autoPitchTarget;
        pitchError = pitch - pitchTarget; // - accelPitchAdjust;   // ADD ACCEL PITCH ADJUSTMENT?

        // Kpitch needs to decrease when the arm is down
        //double newKpitch = Kpitch * Math.cos((Math.PI/180)*theArm.getAngle() / 2.6);
        double pitchVolts = Kpitch * pitchError + KpitchRate * pitchRATE;
        double totalPowerVolts = pitchVolts + positionVolts;

        // The following controls the turn (yaw) of the robot
        // getYaw always returns value from -2*PI to 2*PI
        rawYaw = orientation.getYaw(AngleUnit.RADIANS);
        // The code below makes "yaw" a continuous value
        double deltaYaw = rawYaw - rawPriorYaw;
        rawPriorYaw = rawYaw;
        if (deltaYaw > Math.PI) deltaYaw -= 2 * Math.PI;
        else if (deltaYaw < -Math.PI) deltaYaw += 2 * Math.PI;
        yaw = priorYaw + deltaYaw;
        priorYaw = yaw;

        yawPID.setSetpoint(yawTarget);
        yawPower = yawPID.compute(yaw);

        // limit the total volts
        if (totalPowerVolts > 14) totalPowerVolts = 14;
        else if (totalPowerVolts < -14) totalPowerVolts = -14;

         // Set the motor power for both wheels
        leftDrive.setPower(totalPowerVolts / currentVoltage - yawPower + zeroVoltsAdjust);
        rightDrive.setPower(totalPowerVolts / currentVoltage + yawPower + zeroVoltsAdjust);

        if (ClawIsClosed) clawServo.setPosition(1.0); // closed value (0.98 for blocks)
        else clawServo.setPosition(0.35); // open value (WAS 0.35)

        if(APRILTAG) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    at_id = detection.id;
                    at_range = detection.ftcPose.range;
                    at_bearing = detection.ftcPose.bearing;
                }
            }
        }

        if (LOG) {
            // Data log 
            // Note that the order in which we set datalog fields
            // does *not* matter! Order is configured inside the Datalog class constructor.
            datalog.loopCounter.set(i);
            datalog.runTime.set(currentTime);
            datalog.deltaTime.set(deltaTime);
            datalog.pos.set(sOdom);
            datalog.posTarget.set(posTarget);
            datalog.veloTarget.set(veloTarget);
            //datalog.accelTarget.set(accelTarget);
            datalog.pitch.set(pitch);
            datalog.pitchTarget.set(pitchTarget);
            datalog.pitchRATE.set(pitchRATE);
            datalog.yaw.set(yaw);
            datalog.yawTarget.set(yawTarget);
            datalog.yawTheta.set(theta);
            datalog.yawRATE.set(angularVelocity.zRotationRate);
            datalog.x.set(odometry.getX());
            datalog.y.set(odometry.getY());
            datalog.leftTicks.set(leftTicks);
            datalog.rightTicks.set(rightTicks);
            datalog.linVelo.set(odometry.getLinearVelocity());
            datalog.avgLinVelo.set(linearVelocity); // this is a running average
            datalog.positionVolts.set(positionVolts);
            datalog.pitchVolts.set(pitchVolts);
            datalog.totalVolts.set(totalPowerVolts);
            datalog.yawPwr.set(yawPower * currentVoltage);
            datalog.battery.set(battery.getVoltage());
            datalog.armAngle.set(theArm.getAngle());
            datalog.Kpos.set(Kpos);
            datalog.Kvelo.set(Kvelo);
            datalog.Kpitch.set(Kpitch);
            datalog.KpitchRate.set(KpitchRate);
            datalog.PosAmplitude.set(PosAmplitude);
            datalog.PitchAmplitude.set(PitchAmplitude);
            datalog.ATid.set(at_id);
            datalog.ATrange.set(at_range);
            datalog.ATbearing.set(at_bearing);

            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();
        }
        if (TELEMETRY) {
            theOpmode.telemetry.addData("s position Target (mm)", "%.1f ", posTarget);
            theOpmode.telemetry.addData("s position Odometry (mm)", "%.1f ", sOdom);

            theOpmode.telemetry.addData("Velocity Target (mm/sec)", "%.1f ", veloTarget);
            theOpmode.telemetry.addData("Velocity Current (mm/sec)", "%.1f ", linearVelocity);

            theOpmode.telemetry.addData("Arm Target Angle", theArm.getTargetAngle());
            theOpmode.telemetry.addData("Arm Current Angle", theArm.getAngle());

            theOpmode.telemetry.addData("Pitch Target (degrees)", "%.1f ", pitchTarget);
            theOpmode.telemetry.addData("Pitch IMU (degrees)", "%.1f ", pitch);

            theOpmode.telemetry.addData("Claw Servo", clawServo.getPosition());
        }

        // kill the robot if it pitches over or runs fast
        if (pitch > 80  || pitch < -80 || linearVelocity > 1500 || linearVelocity < -1500) {
            theOpmode.requestOpModeStop(); // Stop the opmode
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
        public Datalogger.GenericField deltaTime = new Datalogger.GenericField("deltaTime");
        public Datalogger.GenericField pitch = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField pitchTarget = new Datalogger.GenericField("PitchTarget");
        public Datalogger.GenericField pitchRATE = new Datalogger.GenericField("pitchRATE");
        public Datalogger.GenericField pos = new Datalogger.GenericField("PosCurrent");
        public Datalogger.GenericField posTarget = new Datalogger.GenericField("PosTarget");
        public Datalogger.GenericField veloTarget = new Datalogger.GenericField("VeloTarget");
        public Datalogger.GenericField yaw = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField yawTarget = new Datalogger.GenericField("yawTarget");
        public Datalogger.GenericField yawTheta = new Datalogger.GenericField("Theta");
        public Datalogger.GenericField yawRATE = new Datalogger.GenericField("yawRATE");
        public Datalogger.GenericField x = new Datalogger.GenericField("x");
        public Datalogger.GenericField y = new Datalogger.GenericField("y");
        public Datalogger.GenericField leftTicks = new Datalogger.GenericField("leftTicks");
        public Datalogger.GenericField rightTicks = new Datalogger.GenericField("rightTicks");
        public Datalogger.GenericField linVelo = new Datalogger.GenericField("linearVelo");
        public Datalogger.GenericField avgLinVelo = new Datalogger.GenericField("AvgLinearVelo");
        public Datalogger.GenericField positionVolts = new Datalogger.GenericField("positionVolts");
        public Datalogger.GenericField pitchVolts = new Datalogger.GenericField("pitchVolts");
        public Datalogger.GenericField totalVolts = new Datalogger.GenericField("totalVolts");
        public Datalogger.GenericField yawPwr = new Datalogger.GenericField("yawPower");
        public Datalogger.GenericField battery = new Datalogger.GenericField("Battery");
        public Datalogger.GenericField armAngle = new Datalogger.GenericField("armAngle");
        public Datalogger.GenericField Kpos = new Datalogger.GenericField("Kpos");
        public Datalogger.GenericField Kvelo = new Datalogger.GenericField("Kvelo");
        public Datalogger.GenericField Kpitch = new Datalogger.GenericField("Kpitch");
        public Datalogger.GenericField KpitchRate = new Datalogger.GenericField("KpitchRate");

        public Datalogger.GenericField PosAmplitude = new Datalogger.GenericField("PosAmplitude");
        public Datalogger.GenericField PitchAmplitude = new Datalogger.GenericField("PitchAmplitude");
        public Datalogger.GenericField ATid = new Datalogger.GenericField("AprilTagID");
        public Datalogger.GenericField ATrange = new Datalogger.GenericField("AprilTagRange");
        public Datalogger.GenericField ATbearing = new Datalogger.GenericField("AprilTagBearing");

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
                            deltaTime,
                            pitch,
                            pitchTarget,
                            pitchRATE,
                            pos,
                            posTarget,
                            veloTarget,
                            //accelTarget,
                            yaw,
                            yawTarget,
                            yawTheta,
                            yawRATE,
                            x,
                            y,
                            leftTicks,
                            rightTicks,
                            linVelo,
                            avgLinVelo,
                            positionVolts,
                            pitchVolts,
                            totalVolts,
                            yawPwr,
                            battery,
                            armAngle,
                            Kpos,
                            Kvelo,
                            Kpitch,
                            KpitchRate,
                            PosAmplitude,
                            PitchAmplitude,
                            ATid,
                            ATrange,
                            ATbearing
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }

    /**
     * TWB method to provide buttons for tuning zero voltage constant.
     */
    public void adjustThingButtons() {
        if (theOpmode.gamepad1.leftStickButtonWasPressed())  zeroVoltsAdjust += 0.1;
        else if (theOpmode.gamepad1.rightStickButtonWasPressed()) zeroVoltsAdjust -= 0.1;

        theOpmode.telemetry.addData("STICK BUTTONS", "ADJUST ZERO VOLTS");
        theOpmode.telemetry.addData("Zero Volts Adjust (volts)", "%.1f ", zeroVoltsAdjust);
    }

    /**
     * TWB method to provide buttons for tuning feedback constants.
     */
    public void tuneButtons() {
        if (theOpmode.gamepad1.dpadUpWasPressed()) Kpitch += 0.01;
        else if (theOpmode.gamepad1.dpadDownWasPressed()) Kpitch -= 0.01;
        else if (theOpmode.gamepad1.dpadLeftWasPressed()) Kvelo += 0.001;
        else if (theOpmode.gamepad1.dpadRightWasPressed()) Kvelo -= 0.001;
        else if (theOpmode.gamepad1.yWasPressed()) KpitchRate += 0.001;
        else if (theOpmode.gamepad1.aWasPressed()) KpitchRate -= 0.001;
        else if (theOpmode.gamepad1.xWasPressed()) Kpos += 0.001;
        else if (theOpmode.gamepad1.bWasPressed()) Kpos -= 0.001;

        theOpmode.telemetry.addData("K pos   +X - B", Kpos);
        theOpmode.telemetry.addData("K velo  +LEFT -RIGHT", Kvelo);
        theOpmode.telemetry.addData("K pitch +UP -DOWN", Kpitch);
        theOpmode.telemetry.addData("K pitch rate +Y -A", KpitchRate);
    }

    /**
     * TWB method to provide user control of the arm.
     */
    public void arm_teleop() {
        //Increment target Arm angle
        if (theOpmode.gamepad1.right_stick_y > 0.3) theArm.setArmAngle(theArm.getAngle() + 2.0);
        else if (theOpmode.gamepad1.right_stick_y < -0.3) theArm.setArmAngle(theArm.getAngle() - 2.0);

        //Set arm angle for cargo deposit
        if (theOpmode.gamepad1.left_bumper) theArm.setArmAngle(90.0);

        //Set arm angle to cargo collection
        if (theOpmode.gamepad1.right_bumper) theArm.setArmAngle(-140.0);

        //Sets arm to vertical. Increment and hold are modified to properly represent the operative state
        if (theOpmode.gamepad1.left_trigger > 0.5) theArm.setArmAngle(0.0);
    }

    /**
     * TWB method to provide user control of the claw.
     */
    public void claw_teleop() {
        //Controls the claw boolean
        if ((theOpmode.gamepad1.right_trigger > 0.5) && clawTimer.milliseconds() > 333) {
            clawTimer.reset();
            ClawIsClosed = !ClawIsClosed;

            if (ClawIsClosed) theArm.setArmAngle(theArm.getAngle() + 5.0);  // raise the arm a bit
        }
    }

    /**
     * TWB method to provide user control of turning the robot.
     * @param speed a value from 0.01 to 0.05 to adjust the speed of the turn
     */
    public void turn_teleop(double speed) {
        // Robot Turning:
        // The right joystick turns the robot by adjusting the yaw PID turn setpoint
        yawTarget -= theOpmode.gamepad1.right_stick_x * speed;  // get turn from gamepad (radian)
    }

    /**
     * TWB method to provide user control of moving the robot with joystick, controlling velocity.
     * @param MaxVelo Maximum velocity (positive or negative)
     */
    public void velo_teleop(double MaxVelo) {
        // joystick sets forward and back velocity
        double stickVeloTarget = -theOpmode.gamepad1.left_stick_y * MaxVelo;
        //oldVeloTarget = veloTarget;
        double deltaVelo = stickVeloTarget - veloTarget;
        if (Math.abs(deltaVelo) >= 5) deltaVelo = Math.signum(deltaVelo)*5.1;
        else deltaVelo = 0;

        veloTarget += deltaVelo;
        // check if Max Velocity will be exceeded
        if (veloTarget > MaxVelo) {
            veloTarget = MaxVelo;
        } else  if (veloTarget < -MaxVelo) {
            veloTarget = -MaxVelo;
        }

        posTarget += veloTarget*deltaTime;
    }
    /**
     * TWB method to provide user control of moving the robot with joystick, controlling pitch.
     */
    public void pitch_teleop() {

        // add some pitch to get it moving
        autoPitchTarget = theOpmode.gamepad1.left_stick_y * 8.0;

        // move the position target as well
        posTarget -= theOpmode.gamepad1.left_stick_y * 7.0;
    }
     /*
    public void slow_ramp() {

        // Slow Power Ramp, for Ks determination
        //totalPowerVolts = (double)i * 0.002;
    }
     */
}