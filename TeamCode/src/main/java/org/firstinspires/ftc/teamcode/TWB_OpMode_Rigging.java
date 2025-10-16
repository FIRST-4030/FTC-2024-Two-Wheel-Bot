/* This Iterative Tele OpMode is for rigging a Two Wheel Balancing Robot with Arm
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This Iterative Tele OpMode is for Rigging a Two Wheel Balancing Robot with Arm
 */
@TeleOp(name="TWB Rigging")
//@Disabled
public class TWB_OpMode_Rigging extends OpMode
{
    // Declare OpMode members.
    private TwoWheelBalanceBot twb;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new TwoWheelBalanceBot(hardwareMap,this); // Create twb object

        twb.LOG = true;
        twb.APRILTAG = true;

        twb.init();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("RIGGING", "INIT LOOP");

        twb.init_loop(); // provides user a chance to change the K terms

        twb.servo_rig_loop();

        twb.tuneButtons(); // used to tune the K terms

        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

        twb.start(); // gets the latest state of the robot before running
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        // get teleoperated inputs

        twb.velo_teleop(500); // set robot velocity and position targets

        twb.turn_teleop(0.01); // set robot yaw angle target

        twb.arm_teleop();

        twb.claw_teleop();

        twb.tuneButtons(); // used to tune the K terms

        twb.adjustThingButtons();

        twb.loop();  // MAIN CONTROL SYSTEM

        telemetry.update();
    }
}
