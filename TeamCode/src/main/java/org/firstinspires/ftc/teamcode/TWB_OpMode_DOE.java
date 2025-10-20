/* This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot with Arm
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This Iterative OpMode is for a Two Wheel Balancing Robot with Arm.
 * It initializes to balancing, then runs through a series of Kposition and Kpitch values
 * with small perturbations to make the robot rock.
 * Telemetry shows the values with the lowest oscillations - write these down at the end!
 * Also, one can review the datalog and see when the robot is well balanced.
 */
@TeleOp(name="TWB Design of Experiments")
//@Disabled
public class TWB_OpMode_DOE extends OpMode {
    // Declare OpMode members.
    private TwoWheelBalanceBot twb;
    final private ElapsedTime moveTimer = new ElapsedTime();

    // DOE constants.  Modify these for the experiment
    private final double minKpos = 0.010; //
    private final double maxKpos = 0.020; //
    private final double incrementKpos = (maxKpos-minKpos) / 5.0;  // calculated
    private final double minKpitch = -0.60; // -.85 for -90 to 90
    private final double maxKpitch = -0.50; // -0.70 for -90 to 90
    private final double incrementKpitch = (maxKpitch-minKpitch) / 5.0; // calculated

    private double armAngle = -90.0; // degrees

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
    private double priorMinPitchAmp = 15; // used for datalogging

    int count = 1; // for counting the DOE

    boolean dirPos = true;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new TwoWheelBalanceBot(hardwareMap,this); // Create twb object

        twb.LOG = true;
        twb.TELEMETRY = false;
        twb.APRILTAG = false;

        KPOS = minKpos;
        KPIT = minKpitch;
        //twb.Kpos = KPOS;
        twb.Kvelo = KPOS;
        twb.Kpitch = KPIT;
        twb.PosAmplitude = priorMinPosAmp; // for datalogging
        twb.PitchAmplitude = priorMinPitchAmp; // for datalogging

        twb.ClawIsClosed = true; // close the claw

        twb.init();
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("AUTO", "INIT LOOP");
        telemetry.addData("Adjust arm angle for ","Design of Experiments");

        if (gamepad1.xWasPressed()) armAngle += 10;
        else if (gamepad1.bWasPressed()) armAngle -= 10;
        twb.theArm.setArmAngle(armAngle);  // sets the arm angle, does not move it

        telemetry.addData("Move Arm  + X - B", armAngle);

        double fixedPitchTarget = twb.theArm.updateArm(0.02); // moves the arm
        telemetry.addData("Robot Pitch should be ", fixedPitchTarget);

        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        resetRuntime();

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
        double period = 5.0; // seconds
        double jiggleDist = 50.0;  // mm for jiggle

        if (getRuntime() < 4.0) { // initialization
            twb.loop();  // MAIN CONTROL SYSTEM
            telemetry.addData("Initialization","Settle");
            telemetry.update();
            moveTimer.reset();
            twb.posTarget = jiggleDist;
        } else {

            // Outer Loop.
            if(KPOS < maxKpos) {
                //twb.Kpos = KPOS;
                twb.Kvelo = KPOS;
                twb.Kpitch = KPIT;

                twb.loop();  // MAIN CONTROL SYSTEM

                // give robot a jiggle at the beginning of the period to get a wave
//
                if(moveTimer.seconds() < 0.1) {
                    twb.autoPitchTarget = 3.0; // add X degrees initially to jiggle
                } else {
                    twb.autoPitchTarget = 0.0;
                }
//

                //if (moveTimer.seconds() > 0.2) {
                    // build the minimum amplitude "box" on the position wave
                    minPos = Math.min(twb.sOdom,minPos);
                    maxPos = Math.max(twb.sOdom,maxPos);
                    posAmp = maxPos-minPos;

                    // build the minimum amplitude "box" on the pitch wave
                    minPitch = Math.min(twb.pitch,minPitch);
                    maxPitch = Math.max(twb.pitch,maxPitch);
                    pitchAmp = maxPitch-minPitch;
                //}
                // at end of period, check for a new minimum and save it
                // Position wave
                if ((posAmp < minPosAmp) && (moveTimer.seconds() > period-0.05)) {
                    minPosAmp = posAmp;
                    minPosAmpKpos = KPOS;
                    minPosAmpKpitch = KPIT;
                }

                // at end of period, check for a new minimum and save it
                // Pitch wave
                if ((pitchAmp < minPitchAmp) && (moveTimer.seconds() > period-0.05)) {
                    minPitchAmp = pitchAmp;
                    minPitchAmpKpos = KPOS;
                    minPitchAmpKpitch = KPIT;
                }

                // Inner loop
                if (moveTimer.seconds() > period) {

                    // Inner loop direction switch
                    if ((KPIT > maxKpitch) || (KPIT < minKpitch)) {
                        KPIT_REV=!KPIT_REV;
                        KPOS = KPOS + incrementKpos;
                    }

                    if (KPIT_REV)  KPIT=KPIT-incrementKpitch;
                    else if (count != 1)  KPIT=KPIT+incrementKpitch;

                    moveTimer.reset();

                    // jiggle the target position each period
                    dirPos = !dirPos;
                    //if (dirPos) twb.posTarget = jiggleDist;
                    //else twb.posTarget = -jiggleDist;

                    count += 1;
                    minPos = 1000;
                    maxPos = -1000;
                    minPitch = 45;
                    maxPitch = -45;
                    priorMinPosAmp = posAmp;      // for datalogging
                    priorMinPitchAmp = pitchAmp;  // for datalogging
                }

                twb.PosAmplitude = priorMinPosAmp; // for datalogging
                twb.PitchAmplitude = priorMinPitchAmp; // for datalogging

                telemetry.addData("COUNT",count);
                telemetry.addData("Kposition",KPOS);
                telemetry.addData("Kpitch",KPIT);
                telemetry.addData("Pos Wave Min Kpos", minPosAmpKpos);
                telemetry.addData("Pitch Wave min Kpos", minPitchAmpKpos);
                telemetry.addData("Pos Wave Min Kpitch", minPosAmpKpitch);
                telemetry.addData("Pitch Wave min Kpitch", minPitchAmpKpitch);
                telemetry.update();

            } else {
                twb.loop();  // MAIN CONTROL SYSTEM

                telemetry.addData("FINAL COUNT",count);
                telemetry.addData("FINAL Position Wave Min Kpos", minPosAmpKpos);
                telemetry.addData("FINAL Pitch Wave min    Kpos", minPitchAmpKpos);
                telemetry.addData("FINAL Pos Wave Min   Kpitch", minPosAmpKpitch);
                telemetry.addData("FINAL Pitch Wave min Kpitch", minPitchAmpKpitch);

                telemetry.update();

                if (moveTimer.seconds() > 8.0) requestOpModeStop(); // Stop the opmode
            }
        }

    }
}