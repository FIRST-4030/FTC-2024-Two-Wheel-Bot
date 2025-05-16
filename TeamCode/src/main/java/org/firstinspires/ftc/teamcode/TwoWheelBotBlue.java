/*
This FTC OpMode is for a Two Wheel Balancing Robot
This OpMode is extends Iterative OpMode (not LinearOpMode)
Drive Mode: the Right stick moves the robot fwd and back, the Left stick turns left and right.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@TeleOp(name = "2WheelRobot-BlueWheels")
public class TwoWheelBotBlue extends OpMode
{
    boolean LOG=false;  // should the log be recorded?
    
    Datalog datalog; // create the data logger
    
    // NEW FEEDBACK CONSTANTS
    double Kvelo = 0.013;  // volts/mm/sec   
    double Kpitch = -0.165; // volts/degree
    double KpitchRate = -0.016; // volts/degrees/sec
    double Kpos = 0.011;  // volts/mm

    static double Ks = 0.2; // Feedforward Static constant, volts
    //static double Kv = 0.0051; // Feedforward Velo constant, volts/mm/sec
    //static double Kp = 0.010; // Velo error proportional constant, volts
    //static double maxLinVelo = 200.0; // maximum linear velocity
    //double maxDeltaVelo = 1.0; // max delta linear velocity

    // YAW PID (presently same for both robots)
    PIDController yawPID = new PIDController(0.4,0.1,0.05); // kp, ki, kd
    
    private IMU imu; // IMU on Control Hub
    // Object are part of FIRST navigation classes
    private YawPitchRollAngles orientation;   
    private AngularVelocity angularVelocity; 

    double pitch = 0;
    double pitchRATE = 0; 
    RunningAverage pitchRateRA;   // USES THE RunningAverage class

    double totalPowerVolts = 0;
    double positionVolts = 0; 
    double pitchVolts = 0;
    
    double yaw = 0;
    double priorYaw = 0;
    double rawYaw, rawPriorYaw = 0;
    double yawRATE, yawPower;

    private VoltageSensor battery;
    double currentVoltage = 0.0;
    
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    static final double TICKSPERMM = 1.7545; // 2.3393; // blue, REV SPUR 40:1, 6in wheels
    //static final double TICKSPERMM = 1.428; // BLACK, Gobilda 5203 12.2:1, 4.72 dia wheels
    static final double WHEELBASE = 330.0; // blue robot Wheel base
    //static final double WHEELBASE = 325.0; // black
    static final double WHEELDIA = 200.0; //152.4; // blue wheel diameter
    //static final double WHEELDIA = 120.0; // BLACK wheel diameter
    
    private OdometryRA odometry;
    
    // Timers
    private ElapsedTime runtime= new ElapsedTime();  // used with datalog
    private ElapsedTime turnTimer = new ElapsedTime();
    private ElapsedTime distTimer = new ElapsedTime(); // also used for buttons
    double currentTime;
    double lastTime = 0;  // Last timestamp

    int i=0;  // loop counter

    double deltaBalAng=0.0; // Used with some math to control robot pitch
    double turn=0.0;
    double linearVelocity=0.0;  // robots linear velocity, used in PID
    double x, y, s, theta;  // used with odometry
    
    double posTarget = 0;
    double posError = 0;
    
    double pitchTarget = 0;
    double pitchError = 0;
    
    // Two Wheel Robot Move Member
    private TWBMoves myTWBmoves = new TWBMoves(this);
    
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
        battery = hardwareMap.voltageSensor.get("Control Hub"); // BLACK
        
        imu = hardwareMap.get(IMU.class, "imu"); // Control Hub IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
    
        // Initialize the PIDs
        yawPID.setSetpoint(0.0);    // initial yaw (turn) is zero.

        // Initialize Pitch Rate running average
        pitchRateRA = new RunningAverage(7); 

        runtime.reset();

        while( runtime.seconds() < 2.0 || Math.abs(pitch) > 10.0) {
            orientation = imu.getRobotYawPitchRollAngles();
            pitch = orientation.getPitch(AngleUnit.DEGREES);
            telemetry.addData("Pitch (DEG)", pitch);
            telemetry.addData("PITCH MUST BE ","CLOSE TO ZERO TO START");
            telemetry.update();
        }
        
        if (LOG) {
            // Initialize the datalog
            datalog = new Datalog("twoWheelLogBLUE");
    
            // You do not need to fill every field of the datalog
            // every time you call writeLine(); those fields will simply
            // contain the last value.
            datalog.opModeStatus.set("INIT");
            // Write PID constants to the log file. Not using intended fields, because this is done once at start
            datalog.pitch.set(Kpos);       
            datalog.posSetPoint.set(Ks);     
            datalog.pitchRATE.set(Kvelo); 
            datalog.battery.set(battery.getVoltage());
            datalog.yaw.set(yawPID.getKp());
            datalog.yawOdo.set(yawPID.getKi()); 
            datalog.yawRATE.set(yawPID.getKd());
            datalog.writeLine();
        }
        
        telemetry.addData("BLUE BOT READY!"," PRESS |> FOR FUN");
        telemetry.update();

     }
     
    @Override
    public void start() {
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        imu.resetYaw();  // set the yaw value to zero
        
        odometry = new OdometryRA(WHEELBASE,WHEELDIA,pitch); // start the odometry
        
        // reset the timers for the datalog timestamp, turns, runToPos
        runtime.reset();
        turnTimer.reset();
        distTimer.reset();
        
        // reset the PIDs
        yawPID.reset();
        
        // TODO: Update currentVoltage every 10 seconds 
        currentVoltage = battery.getVoltage(); // get battery voltage
    }
    
    // MAIN LOOP
    @Override
    public void loop() {
        // Time... for the deltaTime log
        lastTime = currentTime;
        currentTime = runtime.seconds();
        
        // read wheel motor encoders
        double leftTicks = leftDrive.getCurrentPosition();
        double rightTicks = rightDrive.getCurrentPosition();
        
        i++;  // index the loop counter
        
        orientation = imu.getRobotYawPitchRollAngles();
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        pitch = orientation.getPitch(AngleUnit.DEGREES);

        // get pitch rate term.  Need running average to smooth
        pitchRATE = angularVelocity.xRotationRate;
        pitchRateRA.addNumber(pitchRATE); // add to the running average
        pitchRATE = pitchRateRA.getAverage(); // replace with RA value

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

        yawRATE = angularVelocity.zRotationRate;

        // Odometry is how we get robot position, velocity and turn (yaw)
        odometry.update(leftTicks/TICKSPERMM,rightTicks/TICKSPERMM,pitch);
        x = odometry.getX();
        y = odometry.getY();
        s = odometry.getS();
        linearVelocity = odometry.getAvgLinearVelocity(); // running avg of s
        theta = odometry.getTheta(); 

        // Right joystick moves robot fwd and back, by changing position target
        posTarget -= gamepad1.right_stick_y*7; // add mm to position target
        
        myTWBmoves.handleMoveButtons(s); // s
        
        //myTWBmoves.handleScaleButtons(); // updates distance(mm),time(sec)
        
        posTarget = myTWBmoves.getPosTarget(posTarget);
        pitchTarget = myTWBmoves.getPitchTarget(); 
        
        tuneButtons(); // used to tune the K terms
        
        // MAIN CONTROL CODE:
        posError = s-posTarget;
        positionVolts = Kvelo*linearVelocity - Ks*Math.signum(posError) + Kpos*posError;
        pitchError = pitch -pitchTarget;
        pitchVolts = Kpitch*pitchError + KpitchRate*pitchRATE;

        totalPowerVolts = pitchVolts +  positionVolts;

        // Robot turns by adjusting the yaw PID setpoint
        // bumpers provide 90 degree instant turns
        // joystick provides smooth turns
        //
        if ((gamepad1.left_trigger > 0.5) && turnTimer.seconds() > 0.6) {
            turn += Math.PI/2.0;
            turnTimer.reset();
        } else if ((gamepad1.right_trigger > 0.5) && turnTimer.seconds() > 0.6) {
            turn -= Math.PI/2.0;
            turnTimer.reset();
        } else { 
            // joystick turns the robot by adjusting the yaw PID setpoint
            turn  -=  gamepad1.left_stick_x*0.04;  // get turn from gamepad (radian delta)
        }
        yawPID.setSetpoint(turn); 
        yawPower = yawPID.compute(yaw);

        // Slow Power Ramp, for Ks determination
        //motorPowerVolts = (double)i * 0.002;

        // Set the motor power.  currentVoltage determined at start
        leftDrive.setPower(totalPowerVolts/currentVoltage-yawPower); 
        rightDrive.setPower(totalPowerVolts/currentVoltage+yawPower); 
        
        if (LOG) {
            
            // Data log 
            // Note that the order in which we set datalog fields
            // does *not* matter! Order is configured inside the Datalog class constructor.
            datalog.opModeStatus.set("RUNNING");
            datalog.loopCounter.set(i);
            datalog.runTime.set(currentTime);
            datalog.deltaTime.set(currentTime-lastTime);
            datalog.pitch.set(pitch);
            datalog.posSetPoint.set(posTarget);
            datalog.pitchSetPoint.set(pitchTarget);
            datalog.pitchRATE.set(pitchRATE);
            datalog.yaw.set(yaw);
            datalog.yawOdo.set(turn);
            datalog.yawTheta.set(theta);
            datalog.yawRATE.set(angularVelocity.zRotationRate);
            datalog.x.set(x);
            datalog.y.set(y);
            datalog.s.set(s);
            datalog.leftTicks.set(leftTicks);
            datalog.rightTicks.set(rightTicks);
            datalog.linVelo.set(odometry.getLinearVelocity());
            datalog.avgLinVelo.set(linearVelocity);
            datalog.error.set(posError);
            datalog.posPwr.set(positionVolts); 
            datalog.pitchPwr.set(pitchVolts);
            datalog.totalPwr.set(totalPowerVolts); // PID output
            datalog.yawPwr.set(yawPower);  // PID output
            datalog.battery.set(currentVoltage);
            
            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();
        }
        telemetry.addData("Pitch Target", pitchTarget);
        telemetry.addData("Pitch (DEG)", pitch);
        telemetry.addData("S TARGET (mm)", posTarget);
        telemetry.addData("S (mm)", s);
        telemetry.addData("Robot Yaw (DEG)",turn*180/Math.PI);
        telemetry.addData("Elapsed Time",currentTime);
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
        public Datalogger.GenericField posSetPoint  = new Datalogger.GenericField("S SetPoint");
        public Datalogger.GenericField pitchSetPoint= new Datalogger.GenericField("Pitch SetPoint");
        public Datalogger.GenericField pitchRATE    = new Datalogger.GenericField("pitchRATE");
        public Datalogger.GenericField yaw          = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField yawOdo       = new Datalogger.GenericField("Turn"); // WAS yawOdo
        public Datalogger.GenericField yawTheta     = new Datalogger.GenericField("Theta"); 
        public Datalogger.GenericField yawRATE      = new Datalogger.GenericField("yawRATE");
        public Datalogger.GenericField x            = new Datalogger.GenericField("x");
        public Datalogger.GenericField y            = new Datalogger.GenericField("y");
        public Datalogger.GenericField s            = new Datalogger.GenericField("s");
        public Datalogger.GenericField leftTicks    = new Datalogger.GenericField("leftTicks");
        public Datalogger.GenericField rightTicks   = new Datalogger.GenericField("rightTicks");
        public Datalogger.GenericField linVelo      = new Datalogger.GenericField("linearVelo");
        public Datalogger.GenericField avgLinVelo   = new Datalogger.GenericField("AvgLinear Velo");
        public Datalogger.GenericField error        = new Datalogger.GenericField("S error");
        public Datalogger.GenericField posPwr      = new Datalogger.GenericField("Position Volts");
        public Datalogger.GenericField pitchPwr     = new Datalogger.GenericField("Pitch Volts");
        public Datalogger.GenericField totalPwr     = new Datalogger.GenericField("total Volts");
        public Datalogger.GenericField yawPwr       = new Datalogger.GenericField("yawPwr");
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
                            deltaTime,
                            runTime,
                            posSetPoint,
                            s,
                            pitchSetPoint,
                            pitch,
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
                            avgLinVelo,
                            error,
                            posPwr,
                            pitchPwr,
                            totalPwr,
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
    /*
    Method to call sleep from within a sub method (not in OpMode)
    */
    public void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e) {}
    }

}
