/*
This FTC OpMode is for a Two Wheel High Balancing Robot
With Arm
This OpMode is extends Iterative OpMode (not LinearOpMode)
Drive Mode: the Left stick moves the robot fwd and back, the Right stick turns.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//@Disabled
@TeleOp(name = "2WheelBot_HighCG")
public class OpTwoWheelBotHigh extends OpMode
{
    boolean LOG=true;  // should the log be recorded?

    boolean TUNE=true; // switch for buttons

    Datalog datalog; // create the data logger object

    static double Ks = 0.0;  // Motor Feedforward Static (friction) constant, volts to start moving

    // These are the state terms for a two wheel balancing robot
    double Kpitch = -0.6; // volts/degree
    double KpitchRate = -0.025; // volts/degrees/sec

    double Kpos = 0.04;  // volts/mm For high balancing (unstable) this term is positive
    double Kvelo = 0.015;  // volts/mm/sec For high balancing (unstable) this term is positive

    static final double maxSVelo = 300.0; // max linear velocity (s is any direction). Robot can probably do 1000
    //double veloTarget = 0;

    double posTarget = 0;

    // Robot Body Position (s) Term object. Position limits are not used.
    TWBMotionControl posBody = new TWBMotionControl(0.0,10.0, -10.0,
            maxSVelo, 0.0, 500.0);
    double pitch = 0; // degree
    //private final RunningAverage pitchRateRA = new RunningAverage(3);

    // YAW PID
    PIDController yawPID = new PIDController(0.45,0.12,0.05); // kp, ki, kd

    double yaw = 0;
    double priorYaw = 0;
    double rawYaw, rawPriorYaw = 0;
    double yawRATE, yawPower;
    
    private IMU imu;
    private YawPitchRollAngles orientation;   // part of FIRST navigation classes
    private AngularVelocity angularVelocity;  // part of FIRST navigation classes
    
    private VoltageSensor battery;
    double currentVoltage = 12.0;

    //Handles the arm control, and adjusting the arm for the pitch of the robot
    private TWBArmServo theArm;

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    static final double TICKSPERMM = 1.7545; // blue, REV SPUR 40:1, 8in wheels
    static final double WHEELBASE = 300.0; // blue robot Wheel base (mm)
    static final double WHEELDIA = 203.0; // blue 8 inch wheel diameter (mm)

    TWBOdometry odometry; // two wheel odometry object with running average
    
    // Timers
    ElapsedTime runtime= new ElapsedTime();
    ElapsedTime distTimer = new ElapsedTime();
    private final RunningAverage deltaTimeRA = new RunningAverage(5); // Running average of linear velocity
    private double currentTime;

    double turn=0.0;
    double sOdom, theta;  // used with odometry

    double pitchTarget = 0; // degrees
    double pitchError = 0;

    //private TWBMoves myTWBmoves = new TWBMoves(this);

    //Timer to restrict how frequently the claw opens the closes
    ElapsedTime clawTimer;
    Servo clawServo;

    //Claw boolean
    boolean isClawOpen = false;

    @Override
    public void init() {

        // Initialize the datalog
        if (LOG) datalog = new Datalog("TwoWheelBotHigh");

        deltaTimeRA.addNumber(0.1); // add a number to the running average to avoid divide by zero

        // Define and Initialize Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);  
        rightDrive.setDirection(DcMotor.Direction.FORWARD); 
        
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Get devices from the hardwareMap.
        // as needed, change "Control Hub" to (e.g.) "Expansion Hub 1".
        //battery = hardwareMap.voltageSensor.get("Expansion Hub 1"); // Blue
        battery = hardwareMap.voltageSensor.get("Control Hub"); // BLACK
        
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        //Initialize the arm class, handles arm adjustment
        // ARM LIMITS ARE DEFINED IN ArmServoTWB class
        theArm = new TWBArmServo(hardwareMap, "arm_servo", 0.0);
        // NOTE: Set arm angle to zero to rig the servo (so it is easy to see)
        theArm.setArmAngle(0.0); // initialize angle to collection point, so robot can self balance
        pitchTarget = theArm.updateArm( 0.02); // This will make the arm move

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawTimer = new ElapsedTime();

        yawPID.setSetpoint(0.0);    // initial yaw (turn) is zero.
     }

     // init loop
    @Override
    public void init_loop() {
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        telemetry.addData("Pitch (DEG)", pitch);
        telemetry.addData("PITCH SHOULD BE ","CLOSE (+/-5) TO ZERO TO START");

        if(TUNE) tuneButtons(); // used to tune the K terms

        telemetry.update();
    }

    // start is run once on Start press
    @Override
    public void start() {
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        imu.resetYaw(); // set the yaw value to zero

        odometry = new TWBOdometry(WHEELBASE,WHEELDIA,pitch); // start the odometry
        
        // reset the timers for the datalog timestamp, turns, runToPos
        runtime.reset();
        distTimer.reset();
        
        // reset the PIDs
        yawPID.reset();

        // Get the current voltage just before loop, so that balance control is more consistent
        // Note: getting voltage in loop() can cause feedback issues
        currentVoltage = battery.getVoltage();

        currentTime = runtime.seconds(); // initialize current time for delta time values
    }
    
    // MAIN LOOP
    @Override
    public void loop() {

        double leftTicks = leftDrive.getCurrentPosition();
        double rightTicks = rightDrive.getCurrentPosition();

        int i=0;  // loop counter, used with data logging
        i++;  // index the loop counter

        // compute a loop time.  Using running average to smooth values
        double lastTime = currentTime;
        currentTime = runtime.seconds();
        double deltaTime = currentTime-lastTime;
        // add the new delta time to the running average
        deltaTimeRA.addNumber(deltaTime);
        deltaTime = deltaTimeRA.getAverage(); // replace deltaTime with running average delta time

        // get values from the IMU
        orientation = imu.getRobotYawPitchRollAngles();
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        double pitchRATE = angularVelocity.xRotationRate;
        //pitchRateRA.addNumber(pitchRATE);
        //pitchRATE = pitchRateRA.getAverage(); // replace with running average to smooth
        yawRATE = angularVelocity.zRotationRate;

        // get values from wheel encoders (odometry)
        odometry.update(leftTicks/TICKSPERMM,rightTicks/TICKSPERMM,pitch,deltaTime);
        double linearVelocity = odometry.getAvgLinearVelocity();
        sOdom = odometry.getS();
        theta = odometry.getTheta(); // should be the same value as imu yaw

        // Right joystick moves robot fwd and back, by changing velocity target
        //veloTarget = -gamepad1.left_stick_y*maxSVelo;
        //posBody.setTargetRate(-gamepad1.left_stick_y*maxSVelo);
        posTarget -= gamepad1.left_stick_y*10;
        posBody.setTargetPos(posTarget); //

        posBody.setCurrentPos(sOdom); // need to let the limiter know of current position

        // This updates the target position and limits the velocities
        //posBody.updateVelocity(deltaTime);
        posBody.updatePosition(deltaTime, false);


        // add mm to position target, based on the velocity target
        //posTarget += veloTarget*deltaTime;

        // This method does the auto moves in teleop
        //myTWBmoves.handleMoveButtons(100); // mm move
        //posTarget = myTWBmoves.getPosTarget(posTarget);
        //pitchTarget = myTWBmoves.getPitchTarget();

        if(TUNE) tuneButtons(); // used to tune the K terms

        // Code to adjust terms based on arm angle (robot configuration)

        // MAIN BALANCE CONTROL CODE:
        double posError = sOdom - posBody.getTargetPos();  // TARGET OR CURRENT?
        double veloError = linearVelocity - posBody.getTargetRate(); // TARGET OR CURRENT?
        double positionVolts = Kvelo*veloError - Ks*Math.signum(posError) + Kpos*posError;
        pitchError = pitch - pitchTarget;
        double pitchVolts = Kpitch*pitchError + KpitchRate*pitchRATE;
        double totalPowerVolts = pitchVolts +  positionVolts;

        // Robot Turning:
        // The right joystick turns the robot by adjusting the yaw PID setpoint
        turn -=  gamepad1.right_stick_x*0.02;  // get turn from gamepad (radian delta)

        // The following controls the turn (yaw) of the robot
        // getYaw always returns value from -2*PI to 2*PI
        rawYaw = orientation.getYaw(AngleUnit.RADIANS);
        // The code below makes "yaw" a continuous value
        double deltaYaw = rawYaw-rawPriorYaw;
        rawPriorYaw = rawYaw;
        if (deltaYaw > Math.PI) deltaYaw -= 2*Math.PI;
        else if (deltaYaw < -Math.PI) deltaYaw += 2*Math.PI;
        yaw = priorYaw + deltaYaw;
        priorYaw = yaw;

        yawPID.setSetpoint(turn); 
        yawPower = yawPID.compute(yaw);  // theta is from odometry. yaw is from imu. was yaw

        // Slow Power Ramp, for Ks determination
        //totalPowerVolts = (double)i * 0.002;
        
        // Set the motor power for both wheels
        leftDrive.setPower(totalPowerVolts/currentVoltage-yawPower);
        rightDrive.setPower(totalPowerVolts/currentVoltage+yawPower);

        //Increment target Arm angle
        if (gamepad1.right_stick_y > 0.3) theArm.setArmAngle(theArm.getAngle()+2.0);
        else if (gamepad1.right_stick_y < -0.3) theArm.setArmAngle(theArm.getAngle()-2.0);

        //Set arm angle for cargo deposit
        if (gamepad1.left_bumper) theArm.setArmAngle(90.0);

        //Set arm angle to cargo collection
        if (gamepad1.right_bumper) theArm.setArmAngle(-130.0);

        //Sets arm to vertical. Increment and hold are modified to properly represent the operative state
        if (gamepad1.left_trigger > 0.5) theArm.setArmAngle(0.0);

        // The robot pitch target is determined by the angle of the arm.
        pitchTarget = theArm.updateArm(deltaTime); // update arm position (this makes it move)


        //Controls the claw boolean
        if ((gamepad1.right_trigger > 0.5) && clawTimer.milliseconds() > 333) {
            clawTimer.reset();
            isClawOpen = !isClawOpen;

            if (!isClawOpen)  theArm.setArmAngle(theArm.getAngle()+10.0);  // raise the arm a bit
        }
        if (isClawOpen)  clawServo.setPosition(0.90); // closed value
        else        clawServo.setPosition(0.35); // open value

        if (LOG) {
            // Data log 
            // Note that the order in which we set datalog fields
            // does *not* matter! Order is configured inside the Datalog class constructor.
            datalog.loopCounter.set(i);
            datalog.runTime.set(currentTime);
            datalog.deltaTime.set(deltaTime);
            datalog.pitch.set(pitch);
            datalog.pitchTarget.set(pitchTarget);
            datalog.pitchRATE.set(pitchRATE);
            datalog.yaw.set(yaw);
            datalog.yawOdo.set(turn);
            datalog.yawTheta.set(theta);
            datalog.yawRATE.set(angularVelocity.zRotationRate);
            datalog.x.set(odometry.getX());
            datalog.y.set(odometry.getY());
            datalog.leftTicks.set(leftTicks);
            datalog.rightTicks.set(rightTicks);
            datalog.linVelo.set(odometry.getLinearVelocity());
            datalog.avgLinVelo.set(linearVelocity);
            datalog.veloTarget.set(posBody.getTargetRate());
            datalog.positionVolts.set(positionVolts);
            datalog.pitchVolts.set(pitchVolts);
            datalog.totalVolts.set(totalPowerVolts);
            datalog.yawPwr.set(yawPower*currentVoltage);
            datalog.battery.set(battery.getVoltage());
            datalog.armAngle.set(theArm.getAngle());
            
            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();
        }
        telemetry.addData("s position target (mm)", "%.1f ",posBody.getTargetPos());
        telemetry.addData("s position Odometry (mm)","%.1f ", sOdom);
        telemetry.addData("Velocity target (mm/sec)","%.1f ", posBody.getTargetRate());
        telemetry.addData("Velocity (mm/sec)", "%.1f ",linearVelocity);
        telemetry.addData("Pitch Target (degrees)", pitchTarget);
        telemetry.addData("Pitch (degrees)", pitch);
        telemetry.addData("Arm Target Angle", theArm.getTargetAngle());
        telemetry.addData("Arm Current Angle", theArm.getAngle());
        telemetry.addData("Arm Servo getPosition ", theArm.getPosition());
        telemetry.addData("Claw Servo target ", clawServo.getPosition());
        telemetry.addData("turn setpoint (RADIANS)",turn);
        telemetry.addData("IMU Yaw (RADIANS)",yaw);
        telemetry.addData("Odometry theta (RADIANS)",theta);
        telemetry.update();

    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("LoopCounter");
        public Datalogger.GenericField runTime      = new Datalogger.GenericField("RunTime");
        public Datalogger.GenericField deltaTime    = new Datalogger.GenericField("deltaTime");
        public Datalogger.GenericField pitch        = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField pitchTarget = new Datalogger.GenericField("PitchTarget");
        public Datalogger.GenericField pitchRATE    = new Datalogger.GenericField("pitchRATE");
        public Datalogger.GenericField yaw          = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField yawOdo       = new Datalogger.GenericField("Turn"); // WAS yawOdo
        public Datalogger.GenericField yawTheta     = new Datalogger.GenericField("Theta"); 
        public Datalogger.GenericField yawRATE    = new Datalogger.GenericField("yawRATE");
        public Datalogger.GenericField x    = new Datalogger.GenericField("x");
        public Datalogger.GenericField y    = new Datalogger.GenericField("y");
        public Datalogger.GenericField leftTicks    = new Datalogger.GenericField("leftTicks");
        public Datalogger.GenericField rightTicks    = new Datalogger.GenericField("rightTicks");
        public Datalogger.GenericField linVelo    = new Datalogger.GenericField("linearVelo");
        public Datalogger.GenericField veloTarget    = new Datalogger.GenericField("VeloTarget");
        public Datalogger.GenericField avgLinVelo    = new Datalogger.GenericField("AvgLinearVelo");
        public Datalogger.GenericField positionVolts = new Datalogger.GenericField("positionVolts");
        public Datalogger.GenericField pitchVolts = new Datalogger.GenericField("pitchVolts");
        public Datalogger.GenericField totalVolts = new Datalogger.GenericField("totalVolts");
        public Datalogger.GenericField yawPwr = new Datalogger.GenericField("yawPower");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");
        public Datalogger.GenericField armAngle  = new Datalogger.GenericField("armAngle");

        public Datalog(String name)
        {
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
                            yaw,
                            yawOdo,
                            yawTheta,
                            yawRATE,
                            x,
                            y,
                            leftTicks,
                            rightTicks,
                            linVelo,
                            veloTarget,
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
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
    public void tuneButtons() {
        // buttons for tuning feedback constants
        // NOTE, USING distTimer for length of moves
        double rT = 0.2; // run time in seconds

        if (gamepad1.dpad_up && distTimer.seconds()>rT) {
            Kpitch += 0.01;
            distTimer.reset();
        } else if (gamepad1.dpad_down && distTimer.seconds()>rT) {
            Kpitch -= 0.01;
            distTimer.reset();
        } else if (gamepad1.dpad_left && distTimer.seconds()>rT) {
            Kvelo += 0.001;
            distTimer.reset();
        } else if (gamepad1.dpad_right && distTimer.seconds()>rT) {
            Kvelo -= 0.001;
            distTimer.reset();
        } else if (gamepad1.y && distTimer.seconds()>rT) {
            KpitchRate += 0.001;
            distTimer.reset();
        } else if (gamepad1.a && distTimer.seconds()>rT) {
            KpitchRate -= 0.001;
            distTimer.reset();
        } else if (gamepad1.x && distTimer.seconds()>rT) {
            Kpos += 0.001;
            distTimer.reset();
        } if (gamepad1.b && distTimer.seconds()>rT) {
            Kpos -= 0.001;
            distTimer.reset();
        }

        telemetry.addData("K pos   +X - B", Kpos);
        telemetry.addData("K velo  +LEFT -RIGHT", Kvelo);
        telemetry.addData("K pitch +UP -DOWN", Kpitch);
        telemetry.addData("K pitch rate +Y -A", KpitchRate);
    }
}
