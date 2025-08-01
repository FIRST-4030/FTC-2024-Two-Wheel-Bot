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
    boolean LOG=false;  // should the log be recorded?

    boolean TUNE=false; // switch for buttons

    Datalog datalog; // create the data logger object

    // These are feedforward terms
    static double Ks = 0.2;  // Feedforward Static constant, volts
    static double Kp = 0.01; // Velocity error proportional constant

    // These are the state terms for a two wheel balancing robot
    double Kpitch = -0.245; // volts/degree
    double KpitchRate = -0.0212; // volts/degrees/sec

    double Kpos = 0.0118;  // volts/mm For high balancing (unstable) this term is positive
    double Kvelo = 0.0117;  // volts/mm/sec For high balancing (unstable) this term is positive

    double maxDeltaVelo = 7.0; // max delta linear velocity
    double veloTarget = 0;
    
    // YAW PID
    PIDController yawPID = new PIDController(0.4,0.1,0.05); // kp, ki, kd
    
    double pitch = 0; // degrees
    double pitchRATE = 0;

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
    private ServoHoldPosition stabilizedArm;

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    static final double TICKSPERMM = 1.7545; // blue, REV SPUR 40:1, 8in wheels
    static final double WHEELBASE = 350.0; // blue robot Wheel base (mm)
    static final double WHEELDIA = 203.0; // blue 8 inch wheel diameter (mm)

    OdometryRA odometry; // two wheel odometry object with running average
    
    // Timers
    ElapsedTime runtime= new ElapsedTime();
    ElapsedTime turnTimer = new ElapsedTime();
    ElapsedTime distTimer = new ElapsedTime();
    double currentTime;
    double lastTime = 0;  // Last timestamp
    
    int i=0;  // loop counter, used with data logging

    double turn=0.0;
    double linearVelocity=0.0;  // robots linear velocity, used in PID
    double xOdom, yOdom, sOdom, theta;  // used with odometry
    
    //Target position of the servo, starts at the vertical balance point
    private double servoTarget = 0.209;
    //enum handling the three arm position presets: Vertical, fully down, and partly down. Forward is partly down
    enum ArmPositions {
        UP,
        FORWARD,
        BACKWARD
    }
    ArmPositions armPositions;

    //Code to add pitch correction PID in addition to the encoder PID
    double posTarget = 0; // mm
    double pitchTarget = 0; // degrees
    private TWBMoves myTWBmoves = new TWBMoves(this);
    double posError = 0;
    double positionVolts = 0;
    double pitchError = 0;
    double pitchVolts = 0;
    double totalPowerVolts = 0;

    //Timer to restrict how frequently the claw opens the closes
    ElapsedTime clawTimer;
    Servo clawServo;

    //Claw boolean
    boolean isClawOpen = false;

    @Override
    public void init() {

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
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        //Initial arm position: UP
        armPositions = ArmPositions.UP;

        //Initialize the stabilized arm class, handles arm adjustment
        stabilizedArm = new ServoHoldPosition(hardwareMap, "arm_servo", imu);
        stabilizedArm.update(servoTarget); // cause the arm to move

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawTimer = new ElapsedTime();

        // Initialize the PIDs
        yawPID.setSetpoint(0.0);    // initial yaw (turn) is zero.

        if (LOG) {
            // Initialize the datalog
            datalog = new Datalog("twowheelHighLog");
    
            // You do not need to fill every field of the datalog
            // every time you call writeLine(); those fields will simply
            // contain the last value.
            datalog.opModeStatus.set("INIT");
            // Write PID constants to the log file. Not using intended fields, because this is done once at start
            datalog.pitch.set(Kp);
            datalog.battery.set(battery.getVoltage());
            datalog.yaw.set(yawPID.getKp());
            datalog.yawOdo.set(yawPID.getKi()); 
            datalog.yawRATE.set(yawPID.getKd());
            datalog.writeLine();
        }

     }

    // start is run once on Start press
    @Override
    public void start() {
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        imu.resetYaw(); // set the yaw value to zero

        odometry = new OdometryRA(WHEELBASE,WHEELDIA,pitch); // start the odometry
        
        // reset the timers for the datalog timestamp, turns, runToPos
        runtime.reset();
        turnTimer.reset();
        distTimer.reset();
        
        // reset the PIDs
        yawPID.reset();

        // Get the current voltage just before loop, so that balance control is more consistent
        // Note: getting voltage in loop() can cause feedback issues
        currentVoltage = battery.getVoltage();
    }
    
    // MAIN LOOP
    @Override
    public void loop() {

        double stickVeloTarget = 0;
        
        double leftTicks = leftDrive.getCurrentPosition();
        double rightTicks = rightDrive.getCurrentPosition();
        
        i++;  // index the loop counter
        
        orientation = imu.getRobotYawPitchRollAngles();
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        
        pitch = orientation.getPitch(AngleUnit.DEGREES);

        // The following controls the turn (yaw) of the robot
        // getYaw always returns value from -2*PI to 2*PI
        rawYaw = orientation.getYaw(AngleUnit.RADIANS); 
        // The code below makes "yaw" a continous value
        double deltaYaw = rawYaw-rawPriorYaw;
        rawPriorYaw = rawYaw;
        if (deltaYaw > Math.PI) deltaYaw -= 2*Math.PI;
        else if (deltaYaw < -Math.PI) deltaYaw += 2*Math.PI;
        yaw = priorYaw + deltaYaw;
        priorYaw = yaw;

        pitchRATE = angularVelocity.xRotationRate;

        yawRATE = angularVelocity.zRotationRate;

        odometry.update(leftTicks/TICKSPERMM,rightTicks/TICKSPERMM,pitch);
        linearVelocity = odometry.getAvgLinearVelocity();
        xOdom = odometry.getX();
        yOdom = odometry.getY();
        sOdom = odometry.getS();
        theta = odometry.getTheta(); // should be the same value as imu yaw

        double veloError = stickVeloTarget-veloTarget;

        if (veloError >= 0.1) veloTarget += maxDeltaVelo;
        else if (veloError < 0) veloTarget -= maxDeltaVelo;
        else veloTarget = 0;

        // Robot turns by adjusting the yaw PID setpoint
        // bumpers provide 90 degree instant turns
        // joystick provides smooth turns

        // Right joystick moves robot fwd and back, by changing position target
        posTarget -= gamepad1.left_stick_y*5; // add mm to position target

        // This method does the auto moves in teleop
        //myTWBmoves.handleMoveButtons(100); // mm move

        posTarget = myTWBmoves.getPosTarget(posTarget);
        //pitchTarget = myTWBmoves.getPitchTarget();

        // Buttons for adjusting the pitch target
        if(gamepad1.dpad_up){
            pitchTarget += 0.1;
        }
        if(gamepad1.dpad_down){
            pitchTarget -= 0.1;
        }

        if(TUNE) tuneButtons(); // used to tune the K terms

        // MAIN CONTROL CODE:
        posError = sOdom -posTarget;
        positionVolts = Kvelo*linearVelocity - Ks*Math.signum(posError) + Kpos*posError;
        pitchError = pitch - pitchTarget;
        pitchVolts = Kpitch*pitchError + KpitchRate*pitchRATE;

        totalPowerVolts = pitchVolts +  positionVolts;

        // The right joystick turns the robot by adjusting the yaw PID setpoint
        turn  -=  gamepad1.right_stick_x*0.02;  // get turn from gamepad (radian delta)

        yawPID.setSetpoint(turn); 
        yawPower = yawPID.compute(yaw);  // theta is from odometry. yaw is from imu. was yaw

        // for the deltaTime log
        lastTime = currentTime;
        currentTime = runtime.seconds();
        
        // Slow Power Ramp, for Ks determination
        //totalPowerVolts = (double)i * 0.002;
        
        // Set the motor power.  
        leftDrive.setPower(totalPowerVolts/currentVoltage-yawPower);
        rightDrive.setPower(totalPowerVolts/currentVoltage+yawPower);

        if (!TUNE) {
            //Sets arm to vertical. Increment and hold are modified to properly represent the operative state
            if (gamepad1.y) {
                armPositions = ArmPositions.UP;
                servoTarget = 0.209;
            }

            //Sets arm for cargo collection (90 degrees back)
            if (gamepad1.x) {
                armPositions = ArmPositions.BACKWARD;
                servoTarget = 0.209 + .04;  // control hub pitch is
            }

            //Sets are to be slightly forward for depositing (45 deg angle)
            if (gamepad1.b) {
                armPositions = ArmPositions.FORWARD;
                servoTarget = 0.209 - .04;  // control hub pitch is
            }

            //Controls the claw boolean
            if (gamepad1.a && clawTimer.milliseconds() > 333) {
                clawTimer.reset();
                isClawOpen = !isClawOpen;
            }
            if (isClawOpen) {
                clawServo.setPosition(0.90); // closed value
            } else {
                clawServo.setPosition(0.35); // open value
            }

            //Arm increment with D-pad
            if (gamepad1.dpad_right) {
                servoTarget += 0.0005;
            }
            if (gamepad1.dpad_left) {
                servoTarget -= 0.0005;
            }
        }

        stabilizedArm.update(servoTarget); // update arm position

        if (LOG) {
            // Data log 
            // Note that the order in which we set datalog fields
            // does *not* matter! Order is configured inside the Datalog class constructor.
            datalog.opModeStatus.set("RUNNING");
            datalog.loopCounter.set(i);
            datalog.runTime.set(currentTime);
            datalog.deltaTime.set(currentTime-lastTime);
            datalog.pitch.set(pitch);
            datalog.SetPoint.set(veloTarget);
            datalog.pitchRATE.set(pitchRATE);
            datalog.yaw.set(yaw);
            datalog.yawOdo.set(turn);
            datalog.yawTheta.set(theta);
            datalog.yawRATE.set(angularVelocity.zRotationRate);
            datalog.x.set(xOdom);
            datalog.y.set(yOdom);
            datalog.leftTicks.set(leftTicks);
            datalog.rightTicks.set(rightTicks);
            datalog.linVelo.set(odometry.getLinearVelocity());
            datalog.avgLinVelo.set(linearVelocity);
            datalog.pitchPwr.set(totalPowerVolts); // PID output
            datalog.yawPwr.set(yawPower);  // PID output
            datalog.battery.set(currentVoltage);
            
            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();
        }
        telemetry.addData("Distance Traveled Odometry s (mm)","%.1f ", sOdom);
        telemetry.addData("Linear Velocity (mm/sec)", "%.1f ",linearVelocity);
        telemetry.addData("Total Power Volts", totalPowerVolts);
        telemetry.addData("Pitch Target dpad up-down (degrees)", pitchTarget);
        telemetry.addData("Pitch (degrees)", pitch);
        telemetry.addData("Arm Servo target ", servoTarget);
        telemetry.addData("Arm Servo getPosition ", stabilizedArm.getPosition());
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
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("LoopCounter");
        public Datalogger.GenericField runTime      = new Datalogger.GenericField("RunTime");
        public Datalogger.GenericField deltaTime    = new Datalogger.GenericField("deltaTime");
        public Datalogger.GenericField pitch        = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField SetPoint = new Datalogger.GenericField("SetPoint");
        public Datalogger.GenericField pitchRATE    = new Datalogger.GenericField("pitchRATE");
//        public Datalogger.GenericField pitchIntegral = new Datalogger.GenericField("pitchIntegral");
        public Datalogger.GenericField yaw          = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField yawOdo       = new Datalogger.GenericField("Turn"); // WAS yawOdo
        public Datalogger.GenericField yawTheta     = new Datalogger.GenericField("Theta"); 
        public Datalogger.GenericField yawRATE    = new Datalogger.GenericField("yawRATE");
        public Datalogger.GenericField x    = new Datalogger.GenericField("x");
        public Datalogger.GenericField y    = new Datalogger.GenericField("y");
        public Datalogger.GenericField leftTicks    = new Datalogger.GenericField("leftTicks");
        public Datalogger.GenericField rightTicks    = new Datalogger.GenericField("rightTicks");
        public Datalogger.GenericField linVelo    = new Datalogger.GenericField("linearVelo");
        public Datalogger.GenericField avgLinVelo    = new Datalogger.GenericField("AvgLinearVelo");
        public Datalogger.GenericField error = new Datalogger.GenericField("error");
        public Datalogger.GenericField pitchPwr = new Datalogger.GenericField("PitchPower");
        public Datalogger.GenericField yawPwr = new Datalogger.GenericField("yawPwr");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");

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
                            opModeStatus,
                            loopCounter,
                            runTime,
                            deltaTime,
                            pitch,
                            SetPoint,
                            pitchRATE,
                            //pitchIntegral,
                            yaw,
                            yawOdo,
                            yawTheta,
                            yawRATE,
                            x,
                            y,
                            leftTicks,
                            rightTicks,
                            linVelo,
                            avgLinVelo,
                            error,
                            pitchPwr,
                            yawPwr,
                            battery
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
        // buttons for tunning feedback constants
        // CAUTION, USING DISTTIMER
        double rT = 0.2; // run time in seconds

        if (gamepad1.dpad_up && distTimer.seconds()>rT) {
            Kpitch += 0.01;
            distTimer.reset();
        } else if (gamepad1.dpad_down && distTimer.seconds()>rT) {
            Kpitch -= 0.01;
            distTimer.reset();
        } else if (gamepad1.dpad_left && distTimer.seconds()>rT) {
            Kvelo += 0.0001;
            distTimer.reset();
        } else if (gamepad1.dpad_right && distTimer.seconds()>rT) {
            Kvelo -= 0.0001;
            distTimer.reset();
        } else if (gamepad1.y && distTimer.seconds()>rT) {
            KpitchRate += 0.0001;
            distTimer.reset();
        } else if (gamepad1.a && distTimer.seconds()>rT) {
            KpitchRate -= 0.0001;
            distTimer.reset();
        } else if (gamepad1.x && distTimer.seconds()>rT) {
            Kpos += 0.0001;
            distTimer.reset();
        } if (gamepad1.b && distTimer.seconds()>rT) {
            Kpos -= 0.0001;
            distTimer.reset();
        }

        telemetry.addData("K velo  +LEFT -RIGHT", Kvelo);
        telemetry.addData("K pitch +UP -DOWN", Kpitch);
        telemetry.addData("K pitch rate +Y -A", KpitchRate);
        telemetry.addData("K pos   +X - B", Kpos);
    }
}
