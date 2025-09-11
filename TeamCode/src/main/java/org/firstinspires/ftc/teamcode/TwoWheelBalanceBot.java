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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * Not an OpMode! Top class is for a Two Wheel Balancing Robot With Arm
 */
public class TwoWheelBalanceBot {
    final private OpMode theOpmode; // Set during construction.  Enables using telemetry and gamepad
    public boolean LOG = true;  // should the log be recorded?

    public boolean TUNE = true; // switch for buttons

    Datalog datalog; // create the data logger object

    // These are the state terms for a two wheel balancing robot
    double Kpitch = -0.95; // volts/degree
    double KpitchRate = -0.025; // volts/degrees/sec

    double Kpos = 0.048;  // volts/mm For high balancing (unstable) this term is positive
    double Kvelo = 0.018;  // volts/mm/sec For high balancing (unstable) this term is positive

    static final double TICKSPERMM = 1.7545; // REV SPUR 40:1, 8in wheels
    static final double WHEELBASE = 300; // robot Wheel base (mm)
    static final double WHEELDIA = 203.0; // 8 inch wheel diameter (mm)

    // YAW PID
    PIDController yawPID = new PIDController(0.45, 0.12, 0.05); // kp, ki, kd

    public double posTarget = 0.0;  // from the user joystick in teleop or from auto routines
    double oldPosTarget = 0.0;
    double veloTarget = 0.0;
    double oldVeloTarget = 0.0;
    //double accelTarget = 0.0;
    public double autoPitchTarget = 0; // used to set pitch from an auto routine
    //double accelPitchAdjust = 0.0;
    //private final RunningAverage accelTargetRA = new RunningAverage(3); // Running average of linear

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
    Servo clawServo;

    // Constructor.  Call once in init()
    public TwoWheelBalanceBot(HardwareMap hardwareMap, OpMode opMode) {

        this.theOpmode = opMode; // set the opmode that is calling this class
        // Initialize the datalog
        if (LOG) datalog = new Datalog("TwoWheelBotSep10");

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
        theArm = new TWBArmServo(hardwareMap, "arm_servo", 0.0, 140, -150, 70);
        // NOTE: Set arm angle to zero to rig the servo (so it is easy to see)

        clawServo = hardwareMap.get(Servo.class, "clawServo");

        yawPID.setSetpoint(0.0);    // initial yaw (yawTarget) is zero.
    }

    // init loop
    public void init_loop() {

        if (TUNE) tuneButtons(); // used to tune the K terms
        theOpmode.telemetry.addData("TUNE", TUNE);

        theOpmode.telemetry.addData("LOG", LOG);
    }

    // Gets the robot into a position to self right
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

    // Gets the servos ready for attaching the horns (rigging)
    public void servo_rig_loop() {
        theOpmode.telemetry.addData("ARM", "RIGGING");
        theOpmode.telemetry.addData("ATTACH ARM", "STRAIGHT UP");

        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        theOpmode.telemetry.addData("Pitch (DEG)", pitch);

        theArm.setArmAngle(0.0);
        theOpmode.telemetry.addData("Arm Angle (DEG)", theArm.getAngle());
        armPitchTarget = theArm.updateArm(0.02); // This will make the arm move

        clawServo.setPosition(0.98);
        theOpmode.telemetry.addData("CLAW", "RIGGING");
        theOpmode.telemetry.addData("ATTACH CLAW", "CLOSED");
    }

    // start is run once on Start press
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

    // MAIN LOOP
    // Teleoperated inputs are removed from the main loop method, so loop can be used in auto
    public void loop() {
        double leftTicks = leftDrive.getCurrentPosition();
        double rightTicks = rightDrive.getCurrentPosition();
        AngularVelocity angularVelocity;  // part of FIRST navigation classes

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

        if (TUNE) {
            tuneButtons(); // used to tune the K terms
            adjustThingButtons();
        }

        // MAIN BALANCE CONTROL CODE:
        double posError = sOdom - posTarget;  // Pos Target is from joystick
        double veloError = linearVelocity + veloTarget;  // ADD VELO TARGET
        double positionVolts = Kvelo * veloError + Kpos * posError;
        double sumPitchTarget = armPitchTarget + autoPitchTarget;
        pitchError = pitch - sumPitchTarget; // - accelPitchAdjust;   // ADD ACCEL PITCH ADJUSTMENT
        // Kpitch needs to decrease when the arm is down
        double newKpitch = Kpitch * Math.cos((Math.PI/180)*theArm.getAngle() / 2.6);
        double pitchVolts = newKpitch * pitchError + KpitchRate * pitchRATE;
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
        if (totalPowerVolts > 12) totalPowerVolts = 12;
        else if (totalPowerVolts < -12) totalPowerVolts = -12;

         // Set the motor power for both wheels
        leftDrive.setPower(totalPowerVolts / currentVoltage - yawPower + zeroVoltsAdjust);
        rightDrive.setPower(totalPowerVolts / currentVoltage + yawPower + zeroVoltsAdjust);

        if (ClawIsClosed) clawServo.setPosition(0.98); // closed value (changed .90 to .98)
        else clawServo.setPosition(0.5); // open value (WAS 0.35)

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
            datalog.pitchTarget.set(sumPitchTarget);
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

            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();
        }
        theOpmode.telemetry.addData("s position target (mm)", "%.1f ", posTarget);
        theOpmode.telemetry.addData("s position Odometry (mm)", "%.1f ", sOdom);
        theOpmode.telemetry.addData("Velocity (mm/sec)", "%.1f ", linearVelocity);
        theOpmode.telemetry.addData("Arm Target Angle", theArm.getTargetAngle());
        //theOpmode.telemetry.addData("Arm Current Angle", theArm.getAngle());
        //theOpmode.telemetry.addData("Arm Servo getPosition ", theArm.getPosition());
        theOpmode.telemetry.addData("Pitch IMU (degrees)", "%.1f ", pitch);
        theOpmode.telemetry.addData("Pitch Target (degrees)", "%.1f ", sumPitchTarget);

        // kill the robot if it pitches over or runs fast
        if (pitch > 80  || pitch < -80 || linearVelocity > 1500 || linearVelocity < -1500) {
            theOpmode.requestOpModeStop(); // Stop the opmode
        }
    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
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
        //public Datalogger.GenericField accelTarget = new Datalogger.GenericField("AccelTarget");
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
                            armAngle
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }

    public void adjustThingButtons() {
        // buttons for tuning feedback constants
        // NOTE, USING distTimer for length of moves
        double rT = 0.2; // run time in seconds

        if (theOpmode.gamepad1.left_stick_button && distTimer.seconds() > rT) {
            zeroVoltsAdjust += 0.1;
            distTimer.reset();
        } else if (theOpmode.gamepad1.right_stick_button && distTimer.seconds() > rT) {
            zeroVoltsAdjust -= 0.1;
            distTimer.reset();
        }
        theOpmode.telemetry.addData("STICK BUTTONS", "ADJUST ZERO VOLTS");
        theOpmode.telemetry.addData("Zero Volts Adjust (volts)", "%.1f ", zeroVoltsAdjust);
    }

    public void tuneButtons() {
        // buttons for tuning feedback constants
        // NOTE, USING distTimer for length of moves
        double rT = 0.2; // run time in seconds

        if (theOpmode.gamepad1.dpad_up && distTimer.seconds() > rT) {
            Kpitch += 0.01;
            distTimer.reset();
        } else if (theOpmode.gamepad1.dpad_down && distTimer.seconds() > rT) {
            Kpitch -= 0.01;
            distTimer.reset();
        } else if (theOpmode.gamepad1.dpad_left && distTimer.seconds() > rT) {
            Kvelo += 0.001;
            distTimer.reset();
        } else if (theOpmode.gamepad1.dpad_right && distTimer.seconds() > rT) {
            Kvelo -= 0.001;
            distTimer.reset();
        } else if (theOpmode.gamepad1.y && distTimer.seconds() > rT) {
            KpitchRate += 0.001;
            distTimer.reset();
        } else if (theOpmode.gamepad1.a && distTimer.seconds() > rT) {
            KpitchRate -= 0.001;
            distTimer.reset();
        } else if (theOpmode.gamepad1.x && distTimer.seconds() > rT) {
            Kpos += 0.001;
            distTimer.reset();
        } else if (theOpmode.gamepad1.b && distTimer.seconds() > rT) {
            Kpos -= 0.001;
            distTimer.reset();
        }

        theOpmode.telemetry.addData("K pos   +X - B", Kpos);
        theOpmode.telemetry.addData("K velo  +LEFT -RIGHT", Kvelo);
        theOpmode.telemetry.addData("K pitch +UP -DOWN", Kpitch);
        theOpmode.telemetry.addData("K pitch rate +Y -A", KpitchRate);
    }

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
    public void claw_teleop() {
        //Controls the claw boolean
        if ((theOpmode.gamepad1.right_trigger > 0.5) && clawTimer.milliseconds() > 333) {
            clawTimer.reset();
            ClawIsClosed = !ClawIsClosed;

            if (ClawIsClosed) theArm.setArmAngle(theArm.getAngle() + 5.0);  // raise the arm a bit
        }
     }
    public void turn_teleop() {
        // Robot Turning:
        // The right joystick turns the robot by adjusting the yaw PID turn setpoint
        yawTarget -= theOpmode.gamepad1.right_stick_x * 0.04;  // get turn from gamepad (radian)
    }
    public void forward_teleop() {
        // Right joystick moves robot forward and back
        // Cube the joystick for fine movement control, and cubing keeps the sign
        // Delta of 6 per loop is about 300 mm/sec (6 x 50 loops/sec = 300)
        double mmPerLoop = 6 * Math.cos((Math.PI/180)*theArm.getAngle() / 2);
        oldPosTarget = posTarget;
        posTarget -= Math.pow(theOpmode.gamepad1.left_stick_y, 3) * mmPerLoop;
        oldVeloTarget = veloTarget;
        veloTarget =  (posTarget-oldPosTarget) / deltaTime;
        // acceleration is very choppy and needs to have a running average applied
        //accelTarget = (veloTarget-oldVeloTarget) / deltaTime;
        //accelTargetRA.addNumber(accelTarget);
        //accelTarget = accelTargetRA.getAverage();
        // pitch adjustment due to acceleration. gravity = 9801 mm/sec^2
        //accelPitchAdjust = 0.1 * (180/Math.PI) * (accelTarget/9801.0); // in degrees
        //if (accelPitchAdjust > 5) accelPitchAdjust = 5;
        //else if (accelPitchAdjust < -5) accelPitchAdjust = -5;

    }
    /*
    public void slow_ramp() {

        // Slow Power Ramp, for Ks determination
        //totalPowerVolts = (double)i * 0.002;
    }
     */
}