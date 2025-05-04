package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
/*
 TWB = Two Wheel Bot
 Moves = methods to drive specified distance in specified time
 */

public class TWBMoves {
    // members
    private OpMode myOpMode; // To enable using gamepad1 and telemetry
    
    private boolean runToRunning = false;
    private double rT = 2.0; // position profile run time in seconds
    private double startingS = 0.0;      // set when button is pushed
    //private double startingTurn;   // set when button is pushed
    private double posShift = 800.0; // (mm) set when button is pushed
    //private double turnShift = 0.0; // set when button is pushed
    private double pitchScaler = 1.0;
    private boolean reverseDir = false; // for running backwards
    
    // PieceWise member for position profiling
    private PiecewiseFunction posVector = new PiecewiseFunction();
    
    // PieceWise member for pitch profiling
    private PiecewiseFunction pitchVector = new PiecewiseFunction();
    
    private ElapsedTime moveTimer= new ElapsedTime();
    private ElapsedTime buttonTimer= new ElapsedTime();

    private double max_v;
    private double max_a;
    private int sign = 1;

    // Constructor
    public TWBMoves(OpMode opMode) {
        
        this.myOpMode = opMode;  // To enable using gamepad, telemetry

        // reset the timers for the datalog timestamp, turns, runToPos
        moveTimer.reset();
        buttonTimer.reset();
        
        // Position profile to travel 1 cm in 1 sec, 
        //   using an approximate Triangle Wave acceleration profile
        posVector.debug = false;
        posVector.addElement(.00,.00);
        posVector.addElement(.05,.01);
        posVector.addElement(.10,.02);
        posVector.addElement(.15,.04);
        posVector.addElement(.20,.08);
        posVector.addElement(.25,.14);
        posVector.addElement(.30,.22);
        posVector.addElement(.35,.31);
        posVector.addElement(.40,.40);
        posVector.addElement(.45,.50);
        posVector.addElement(.50,.60);
        posVector.addElement(.55,.69);
        posVector.addElement(.60,.78);
        posVector.addElement(.65,.86);
        posVector.addElement(.70,.91);
        posVector.addElement(.75,.96);
        posVector.addElement(.80,.98);
        posVector.addElement(.85,.99);
        posVector.addElement(.90,1.00);
        posVector.addElement(.95,1.00);
        posVector.addElement(1.00,1.00);

        // Pitch profile to travel 1 cm in 1 sec, 
        //   using an approximate Triangle Wave acceleration profile
        pitchVector.debug = false;
        pitchVector.addElement(.00,.00);
        pitchVector.addElement(.05,-.7); // (.15,-.28)
        pitchVector.addElement(.25,-.5); // (.15,-.28)
        pitchVector.addElement(.35,-.2); // (.85,.28)
        pitchVector.addElement(.70,0);
        pitchVector.addElement(.80,0.1);
        pitchVector.addElement(.90,0.4);
        pitchVector.addElement(1.00,.00);
    }
    public void handleMoveButtons(double currentS) {
        // buttons for running a set distance in a set time
        myOpMode.telemetry.addData("FWD-LEFT BUMP", "   BACK RIGHT-BUMP");

        // Setting rT (run time - global member) here is bad practice!
        if (myOpMode.gamepad1.left_bumper && !runToRunning && buttonTimer.seconds()>rT) {
            runToRunning = true;
            setPitchScaler();
            startingS = currentS;
            reverseDir = false;
            moveTimer.reset();
            buttonTimer.reset();
        } else if (myOpMode.gamepad1.right_bumper && !runToRunning && buttonTimer.seconds()>rT) {
            runToRunning = true;
            setPitchScaler();
            startingS = currentS;
            reverseDir = true;
            moveTimer.reset();
            buttonTimer.reset();
        } 
    }
    
    public double getPosTarget(double posTarget) {
        double currentPosTarget;
        
        if (reverseDir) sign = -1;
        else sign = 1;
        
        if (runToRunning) {
            // runTo code
            currentPosTarget = sign*posShift*posVector.getY(moveTimer.seconds()/rT) + startingS;
            
            
            if (moveTimer.seconds() >= (rT+0.5)) {
                runToRunning = false;
                currentPosTarget = startingS + sign*posShift;
            }
        } else {
            currentPosTarget = posTarget;
        }
        
        return currentPosTarget;
    }
    
    private void setPitchScaler() {
        max_v = 2.0 * posShift / rT;
        max_a = 4.0 * max_v / rT;
        this.pitchScaler = ( max_a / 8.0) / 10.0; // convert from cm to mm
    }
    
    public double getPitchTarget() {
        double pitchTarget = 0;
        
        if (reverseDir) sign = -1;
        else sign = 1;
        
        // moveTimer must be running when this is called!
        if (runToRunning) {
            pitchTarget = sign*pitchScaler*pitchVector.getY(moveTimer.seconds()/rT);
            if (pitchTarget > 20.0) pitchTarget = 20.0;
            else if (pitchTarget < -20.0) pitchTarget = -20.0;
        } 
        return pitchTarget;
    }
    public void handleScaleButtons() {
        double oldPosShift;
        myOpMode.telemetry.addData("DIST (MM)  +UP -DOWN", posShift);
        myOpMode.telemetry.addData("TIME (SEC)", rT);

        if (myOpMode.gamepad1.dpad_up && buttonTimer.seconds()>0.2) {
            oldPosShift = posShift;
            posShift += 10;
            rT = (posShift/oldPosShift) * rT;
            buttonTimer.reset();
        } else if (myOpMode.gamepad1.dpad_down && buttonTimer.seconds()>0.2) {
            oldPosShift = posShift;
            posShift -= 10;
            if (posShift < 50) posShift = 50; 
            rT = (posShift/oldPosShift) * rT;
            buttonTimer.reset();
        } /*else if (myOpMode.gamepad1.dpad_left && buttonTimer.seconds()>0.2) {
            //rT += 0.1;
            buttonTimer.reset();
        } else if (myOpMode.gamepad1.dpad_right && buttonTimer.seconds()>0.2) {
            //rT -= 0.1;
            buttonTimer.reset();
        } */
        
    }
 
}
