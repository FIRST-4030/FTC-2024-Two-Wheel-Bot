/* This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot with Arm
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot with Arm.
 * Initializes to balancing, then runs through a series of Kposition and Kpitch values.
 * The idea is that one can review the datalog and see when the robot is well balanced.
 */
@Autonomous(name="TWB Auto Design of Experiments")
//@Disabled
public class TWB_OpMode_Auto_DOE extends OpMode {
    // Declare OpMode members.
    private TwoWheelBalanceBot twb;
    final private ElapsedTime moveTimer = new ElapsedTime();

    // DOE constants.  Modify these for the experiment
    private final double minKpos = 0.026;
    private final double maxKpos = 0.034;
    private final double incrementKpos = 0.001;  //
    private final double minKpitch = -0.72;
    private final double maxKpitch = -0.66;
    private final double incrementKpitch = 0.01; //
    private final double armAngle = -120.0; // degrees
    private final double period = 3.0; // seconds

    // Design of experiment variables:
    private double KPOS; // DOE value
    private double KPIT; // DOE value
    private boolean KPIT_REV = false;

    // Variables for recording position wave amplitude
    private double minPos = 1000; // mm
    private double maxPos = -1000; // mm
    private double minPosAmp = 200; // mm
    private double minPosAmpKpos = 0; // record of Kpos when Pos Amp is minimum
    private double minPosAmpKpitch = 0; // record of Kpitch when Pos Amp in minimum
    private double priorMinPosAmp = 100; // used for datalogging.

    // Variables for recording pitch wave amplitude
    private double minPitch = 45; // degrees
    private double maxPitch = -45; // degrees
    private double minPitchAmp = 60; // degrees
    private double minPitchAmpKpos = 0; // record of Kpos when Pitch Amp is minimum
    private double minPitchAmpKpitch = 0; // record of Kpitch when Pitch Amp is minimum
    private double priorMinPitchAmp = 30; // used for datalogging

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new TwoWheelBalanceBot(hardwareMap,this); // Create twb object

        twb.TUNE = false;
        twb.LOG = false;
        twb.TELEMETRY = false;

        KPOS = minKpos;
        KPIT = minKpitch;
        twb.Kpos = KPOS;
        twb.Kpitch = KPIT;
        twb.PosAmplitude = priorMinPosAmp; // for datalogging
        twb.PitchAmplitude = priorMinPitchAmp; // for datalogging

        twb.ClawIsClosed = true; // close the claw
        twb.theArm.setArmAngle(armAngle);  // move the arm

        twb.init();

    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("AUTO", "INIT LOOP");
        telemetry.addData("Arm Angle",armAngle);

        //twb.auto_right_loop(); // gets the robot into a position to self right

        twb.init_loop(); // provides user a chance to change the K terms

        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        resetRuntime();
        //moveTimer.reset();

        twb.start(); // gets the latest state of the robot before running
        twb.ClawIsClosed = true; // close the claw
        twb.theArm.setArmAngle(armAngle);  // move the arm
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double posAmp;
        double pitchAmp;

        if (getRuntime() < 4.0) { // initialization
            twb.loop();  // MAIN CONTROL SYSTEM
            telemetry.addData("Initialization","Settle");
            telemetry.update();

            moveTimer.reset();
        } else {
            twb.LOG = true;

            // Outer Loop.
            if(KPOS <= maxKpos) {
                twb.Kpos = KPOS;
                twb.Kpitch = KPIT;

                twb.loop();  // MAIN CONTROL SYSTEM

                // check for minimum amplitude on the position wave
                minPos = Math.min(twb.sOdom,minPos);
                maxPos = Math.max(twb.sOdom,maxPos);
                posAmp = maxPos-minPos;
                if ((posAmp < minPosAmp) && (moveTimer.seconds() > period-0.1)) {
                    minPosAmp = posAmp;
                    minPosAmpKpos = KPOS;
                    minPosAmpKpitch = KPIT;
                }
                twb.PosAmplitude = priorMinPosAmp; // for datalogging

                // check for minimum amplitude on the pitch wave
                minPitch = Math.min(twb.pitch,minPitch);
                maxPitch = Math.max(twb.pitch,maxPitch);
                pitchAmp = maxPitch-minPitch;
                if ((pitchAmp < minPitchAmp) && (moveTimer.seconds() > period-0.1)) {
                    minPitchAmp = pitchAmp;
                    minPitchAmpKpos = KPOS;
                    minPitchAmpKpitch = KPIT;
                }
                twb.PitchAmplitude = priorMinPitchAmp; // for datalogging

                // Inner loop
                if (moveTimer.seconds() > period) {

                    // Inner loop direction switch
                    if ((KPIT > maxKpitch) || (KPIT < minKpitch)) {
                        KPIT_REV=!KPIT_REV;
                        KPOS = KPOS + incrementKpos;
                    }

                    if (KPIT_REV)  KPIT=KPIT-incrementKpitch;
                    else           KPIT=KPIT+incrementKpitch;

                    moveTimer.reset();
                    minPos = 1000;
                    maxPos = -1000;
                    minPitch = 45;
                    maxPitch = -45;
                    priorMinPosAmp = posAmp;      // for datalogging
                    priorMinPitchAmp = pitchAmp;  // for datalogging
                }

                /*
                if((KPIT <= -0.5) && (KPIT >= -1.0)) {
                    if (KPIT_REV)
                        KPIT=KPIT-0.0025;
                    else
                        KPIT=KPIT+0.0025;
                } else {
                    KPIT_REV=!KPIT_REV;
                    KPOS = KPOS + 0.001;
                    if (KPIT_REV)
                        KPIT=KPIT-0.0025;
                    else
                        KPIT=KPIT+0.0025;
                } */
                telemetry.addData("Kposition",KPOS);
                telemetry.addData("Kpitch",KPIT);
                telemetry.addData("Pos Wave Min Kpos", minPosAmpKpos);
                telemetry.addData("Pitch Wave min Kpos", minPitchAmpKpos);
                telemetry.addData("Pos Wave Min Kpitch", minPosAmpKpitch);
                telemetry.addData("Pitch Wave min Kpitch", minPitchAmpKpitch);
                telemetry.update();

            } else {
                requestOpModeStop(); // Stop the opmode
            }
        }

    }
}