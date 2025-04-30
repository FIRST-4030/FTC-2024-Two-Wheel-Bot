/*
This FTC OpMode is for a Two Wheel Balancing Robot by SrAmo
This OpMode is extends Iterative OpMode (not LinearOpMode)
Drive Mode: the Right stick moves the robot fwd and back, the Left stick turns left and right.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.Timer;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Disabled
@TeleOp(name = "2Wheel_Low Feed Forward")
public class OpTwoWheelBalanceDrive extends OpMode
{
    boolean CAMERA=false; // does the robot have a camera?
    boolean LOG=true;  // should the log be recorded?
    
    Datalog datalog; // create the data logger

    static double Ks = 0.2979; //0.6574; // Feedforward Static constant, volts
    static double Kv = 0.0074; //0.0063; // Feedforward Velo constant, volts/mm/sec
    static double Kp = 0.015; // Velo error proportional constant
    static double maxLinVelo = 600.0; // maximum linear velocity
    double maxDeltaVelo = 7.0; // max delta linear velocity
    double veloTarget = 0;
    int junk=1;
    
    // YAW PID (presently same for both robots)
    PIDController yawPID = new PIDController(0.4,0.1,0.05); // kp, ki, kd
    
    public double pitch = 0;
    public double pitchRATE = 0;
    public double motorPowerVolts = 0;

    public double yaw = 0;
    public double priorYaw = 0;
    public double rawYaw, rawPriorYaw = 0;
    public double yawRATE, yawPower;
    
    private IMU imu;
    private YawPitchRollAngles orientation;   // part of FIRST navigation classes
    private AngularVelocity angularVelocity;  // part of FIRST navigation classes
    
    private VoltageSensor battery;
    private ServoHoldPosition stabilizedArm;
    double currentVoltage = 0.0;
    
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    static final double TICKSPERMM = 1.7545; // 2.3393; // blue, REV SPUR 40:1, 6in wheels
    //static final double TICKSPERMM = 1.428; // BLACK, Gobilda 5203 12.2:1, 4.72 dia wheels
    static final double WHEELBASE = 323.0; // blue robot Wheel base
    //static final double WHEELBASE = 280.0*1.08; // BLACK robot Wheel base (calculated) x 1.08
    static final double WHEELDIA = 200.0; //152.4; // blue wheel diameter
    //static final double WHEELDIA = 120.0; // BLACK wheel diameter
    
    private Odometry odometry;
    
    // Timers
    private ElapsedTime runtime= new ElapsedTime();
    private ElapsedTime turnTimer = new ElapsedTime();
    private ElapsedTime distTimer = new ElapsedTime();
    double currentTime;
    double lastTime = 0;  // Last timestamp
    
    private AprilTagClass myAprilTag;
    private int numTags = 0;

    //DistanceSensor dist1; // back
    //DistanceSensor dist2; // front
    //private DigitalChannel redLED;
    //private DigitalChannel greenLED;
    static double RANGE = 35; // cm
    boolean frontBrake = false;
    boolean backBrake = false;

    int i=0;  // loop counter

    double deltaBalAng=0.0; // Used with some math to control robot pitch
    double turn=0.0;
    double linearVelocity=0.0;  // robots linear velocity, used in PID
    double x, y, theta;  // used with odometry
    
    // PieceWise members, for velocity ramping, for runToPos
    private PiecewiseFunction veloCurve = new PiecewiseFunction();
    boolean runToRunning = false;
    double distance = 1.0; // mm
    double runTime = 1.0; // seconds

    private double servoTarget = 0.5;
    enum ArmPositions {
        UP,
        FORWARD,
        BACKWARD
    }
    ArmPositions armPositions;


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
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, 
    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        armPositions = ArmPositions.UP;
        stabilizedArm = new ServoHoldPosition(hardwareMap, "arm_servo", imu);

        // Initialize the PIDs
        yawPID.setSetpoint(0.0);    // initial yaw (turn) is zero.

        // Get the distance sensor and motor from hardwareMap
        //dist1 = hardwareMap.get(DistanceSensor.class, "dist1");
        //dist2 = hardwareMap.get(DistanceSensor.class, "dist2");

        // Get the LED colors from the hardwaremap
        //redLED = hardwareMap.get(DigitalChannel.class, "red");
        //greenLED = hardwareMap.get(DigitalChannel.class, "green");
        
        //redLED.setMode(DigitalChannel.Mode.OUTPUT);
        //greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        //greenLED.setState(true);
        //redLED.setState(true);

        if (LOG) {
            // Initialize the datalog
            datalog = new Datalog("twowheelLog");
    
            // You do not need to fill every field of the datalog
            // every time you call writeLine(); those fields will simply
            // contain the last value.
            datalog.opModeStatus.set("INIT");
            // Write PID constants to the log file. Not using intended fields, because this is done once at start
            datalog.pitch.set(Kp);       
            datalog.SetPoint.set(Ks);     
            datalog.pitchRATE.set(Kv*100); 
            datalog.battery.set(battery.getVoltage());
            datalog.yaw.set(yawPID.getKp());
            datalog.yawOdo.set(yawPID.getKi()); 
            datalog.yawRATE.set(yawPID.getKd());
            datalog.writeLine();
        }
        // Initialize April Tags
        if (CAMERA) {
            myAprilTag = new AprilTagClass(hardwareMap, this);
    
            myAprilTag.setManualExposure(5, 255);  // Use low exposure time to reduce motion blur
        }

        // Add velocity ramping curve
        veloCurve.debug = false;
        veloCurve.setClampLimits(true);
        veloCurve.addElement(.00,.00);
        veloCurve.addElement(.05,.02);
        veloCurve.addElement(.10,.08);
        veloCurve.addElement(.15,.18);
        veloCurve.addElement(.20,.32);
        veloCurve.addElement(.25,.50);
        veloCurve.addElement(.30,.68);
        veloCurve.addElement(.35,.82);
        veloCurve.addElement(.40,.92);
        veloCurve.addElement(.45,.98);
        veloCurve.addElement(.50,1.00);
        veloCurve.addElement(.55,.98);
        veloCurve.addElement(.60,.92);
        veloCurve.addElement(.65,.82);
        veloCurve.addElement(.70,.68);
        veloCurve.addElement(.75,.50);
        veloCurve.addElement(.80,.32);
        veloCurve.addElement(.85,.18);
        veloCurve.addElement(.90,.08);
        veloCurve.addElement(.95,.02);
        veloCurve.addElement(1.00,.00);

     }
     
    @Override
    public void start() {
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        imu.resetYaw(); // set the yaw value to zero

        odometry = new Odometry(WHEELBASE,WHEELDIA,pitch); // start the odometry
        
        // reset the timers for the datalog timestamp, turns, runToPos
        runtime.reset();
        turnTimer.reset();
        distTimer.reset();
        
        // reset the PIDs
        yawPID.reset();
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
        x = odometry.getX();
        y = odometry.getY();
        theta = odometry.getTheta(); 

        /*
        if (dist1.getDistance(DistanceUnit.CM) > RANGE) {
            redLED.setMode(DigitalChannel.Mode.INPUT); 
            backBrake = false;
            maxDeltaVelo = 7;
        } else {
            redLED.setMode(DigitalChannel.Mode.OUTPUT);
            backBrake = true;
            maxDeltaVelo = 20;
        }
        if (dist2.getDistance(DistanceUnit.CM) > RANGE) {
            greenLED.setMode(DigitalChannel.Mode.INPUT); 
            frontBrake = false;
            maxDeltaVelo = 7;
        } else {
            greenLED.setMode(DigitalChannel.Mode.OUTPUT);
            frontBrake = true;
            maxDeltaVelo = 20;
        } 
        */

        // Right joystick moves robot fwd and back, by changing velocity target
        stickVeloTarget = -gamepad1.right_stick_y * maxLinVelo; // mm/sec
        
        //if (stickVeloTarget > 0 && frontBrake) stickVeloTarget = 0;
        //if (stickVeloTarget < 0 && backBrake) stickVeloTarget = 0;
        
        // buttons for running a set distance
        if (gamepad1.dpad_up && !runToRunning) {
            runToRunning = true;
            distance = 638.0; // mm
            runTime = 2.0;
            distTimer.reset();
        } else if (gamepad1.dpad_down && !runToRunning) {
            runToRunning = true;
            distance = -638.0; // mm
            runTime = 2.0;
            distTimer.reset();
        } else if (gamepad1.dpad_left && !runToRunning){
            runToRunning = true;
            distance = 638.0; // mm
            runTime = 2.0;
            distTimer.reset();
            turn += Math.PI/2.0;
        } if (gamepad1.dpad_right && !runToRunning){
            runToRunning = true;
            distance = 638.0; // mm
            runTime = 2.0;
            distTimer.reset();
            turn -= Math.PI/2.0;
        }
        
        if (runToRunning) {
            // runTo code
            stickVeloTarget = distance*veloCurve.getY(distTimer.seconds()/runTime);
            telemetry.addData("RUN TO RUNNING, TIME = ",distTimer.seconds());
            if (distTimer.seconds() >= (runTime+0.5)) {
                runToRunning = false;
            }
        }
        
        double veloError = stickVeloTarget-veloTarget;
        // velocity ramping
        if (runToRunning) {
            veloTarget = stickVeloTarget;
        } else {
            if (veloError >= 0.1) veloTarget += maxDeltaVelo;
            else if (veloError < 0) veloTarget -= maxDeltaVelo;
            else veloTarget = 0;
        }

        // Feedforward 
        motorPowerVolts = Ks*Math.signum(linearVelocity) + Kv*linearVelocity;
        //motorPowerVolts = Ks*Math.signum(veloTarget) + Kv*veloTarget;
        
        // Feed back, Proportional, add to feedforward
        double error = veloTarget - linearVelocity;
        motorPowerVolts += Kp*error;

        // Robot turns by adjusting the yaw PID setpoint
        // bumpers provide 90 degree instant turns
        // joystick provides smooth turns
        if (gamepad1.left_bumper && turnTimer.seconds() > 0.6) {
            turn += Math.PI/2.0;
            turnTimer.reset();
        } else if (gamepad1.right_bumper && turnTimer.seconds() > 0.6) {
            turn -= Math.PI/2.0;
            turnTimer.reset();
        } else if (gamepad1.a){
            if (CAMERA) {
                // Look for April Tag
                numTags = myAprilTag.getDetections();
                telemetry.addData("April Tags = ",numTags);
                if(numTags>0 && turnTimer.seconds() > 0.05) {
                    turn += myAprilTag.getBearing()/2.0;
                    turnTimer.reset();
                }
            }
        } else {
            // The xxx joystick turns the robot by adjusting the yaw PID setpoint
            turn  -=  gamepad1.left_stick_x*0.05;  // get turn from gamepad (radian delta)
            telemetry.addData("Robot Yaw (RADIANS)",turn);
        }
        yawPID.setSetpoint(turn); 
        yawPower = yawPID.compute(yaw);

        // for the deltaTime log
        lastTime = currentTime;
        currentTime = runtime.seconds();
        
        // Slow Power Ramp, for Ks determination
        //motorPowerVolts = (double)i * 0.002;
        
        // Set the motor power.  
        currentVoltage = battery.getVoltage();
        leftDrive.setPower(motorPowerVolts/currentVoltage-yawPower);
        rightDrive.setPower(motorPowerVolts/currentVoltage+yawPower);

        if(gamepad1.y) {
            armPositions = ArmPositions.UP;
        }
        if(gamepad1.x) {
            armPositions = ArmPositions.BACKWARD;
        }
        if(gamepad1.b) {
            armPositions = ArmPositions.FORWARD;
        }
        if(armPositions.equals(ArmPositions.UP)){
            servoTarget = 0.48;
        } else if(armPositions.equals(ArmPositions.FORWARD)){
            servoTarget = 0.60;
        } else if(armPositions.equals(ArmPositions.BACKWARD)){
            servoTarget = 0.36;
        }

        stabilizedArm.update(servoTarget);

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
            datalog.x.set(x);
            datalog.y.set(y);
            datalog.leftTicks.set(leftTicks);
            datalog.rightTicks.set(rightTicks);
            datalog.linVelo.set(odometry.getLinearVelocity());
            datalog.avgLinVelo.set(linearVelocity);
            datalog.error.set(error);
            datalog.pitchPwr.set(motorPowerVolts); // PID output
            datalog.yawPwr.set(yawPower);  // PID output
            datalog.battery.set(currentVoltage);
            
            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();
        }
/*
        telemetry.addData("Yaw IMU", datalog.yaw);
        telemetry.addData("Yaw Odometry", theta);
        //telemetry.addData("Pitch Motor Power",motorPowerVolts);
        //telemetry.addData("Velo PID d Pitch",veloPIDdPitch);
        //telemetry.addData("Turn Power",turn);
        telemetry.addData("Distance Odometry Y",y);
        //telemetry.addData("OpMode Status", datalog.opModeStatus);
        //telemetry.addData("Loop Counter", datalog.loopCounter);
*/        
        telemetry.addData("Pitch", pitch);
        //telemetry.addData("deltaBalAng",deltaBalAng);
        //telemetry.addData("Pitch Set Point",pitchSetPoint);
        telemetry.addData("Distance Odometry X",x);
        //telemetry.addData("Left Encoder",leftTicks);
        //telemetry.addData("Right Encoder",rightTicks);
        telemetry.addData("Linear Velocity (mm/sec)", linearVelocity);
        telemetry.addData("Stick Velocity (mm/sec)", stickVeloTarget);
        telemetry.addData("Target Velocity (mm/sec)", veloTarget);
        telemetry.addData("Motor Power Volts", motorPowerVolts);
        telemetry.addData("Battery", currentVoltage);
        telemetry.update();

        /*
         * The datalog is automatically closed and flushed to disk after 
         * the OpMode ends - no need to do that manually :')
         */
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
    
    public double constrain(double v, double high, double low) {
        if (v > high)
            return high;
        else if (v < low)
            return low;
        else
            return v;
    }
}
