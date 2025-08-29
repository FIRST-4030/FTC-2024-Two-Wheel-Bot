/* This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot with Arm
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot with Arm
 */
@Autonomous(name="TWB Autonomous 1")
//@Disabled
public class TWB_OpMode_Auto1 extends OpMode {
    // Declare OpMode members.
    private TwoWheelBalanceBot twb;
    final private TWBMoves myTWBmoves = new TWBMoves(); // used for auto
    final private ElapsedTime moveTimer = new ElapsedTime();

    double currentPos;

    enum State {
        START,
        MOVE1,
        MOVE2,
        DONE
    }
    State state = State.START;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new TwoWheelBalanceBot(hardwareMap,this); // Create twb object

        twb.TUNE = true;
        twb.LOG = true;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("AUTO", "INIT LOOP");

        twb.auto_right_loop(); // gets the robot into a position to self right

        twb.init_loop(); // provides user a chance to change the K terms

        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        state = State.START;
        resetRuntime();
        moveTimer.reset();

        twb.start(); // gets the latest state of the robot before running
        twb.ClawIsClosed = true; // close the claw
        twb.theArm.setArmAngle(-90);  // move the arm

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double[] newTargets;
        double DIST = 1000; // mm

        switch (state) {
            case START: // stabilize time
                if (getRuntime() >= 3.0) {
                    state = State.MOVE1;
                    moveTimer.reset();
                    currentPos = 0; // force current pos to zero, for offset in next state
                }
                break;
            case MOVE1:
                if (getRuntime() <= 8.0 ) {
                    newTargets = myTWBmoves.lineMove(DIST,3.0, moveTimer.seconds(),currentPos);
                    twb.posTarget = newTargets[0];
                    twb.autoPitchTarget = newTargets[1];
                } else {
                    state = State.MOVE2;
                    moveTimer.reset();
                    twb.ClawIsClosed = false; // open the claw
                    currentPos = DIST; // for the next state
                }
                break;
            case MOVE2:
                 if (getRuntime() <= 12.0) {
                     myTWBmoves.reverseDir = true;
                    newTargets = myTWBmoves.lineMove(DIST,3.0, moveTimer.seconds(),currentPos);
                    twb.posTarget = newTargets[0];
                    twb.autoPitchTarget = newTargets[1];
                } else {
                    state = State.DONE;
                     moveTimer.reset();
                 }
                break;
            case DONE:
                if (getRuntime() > 13.0 ) requestOpModeStop();
                break;
        }

        twb.loop();  // MAIN CONTROL SYSTEM

        telemetry.addData("State",state);
        telemetry.update();
    }
}
