package org.firstinspires.ftc.teamcode;
/*
 TWB = Two Wheel Bot
 Moves = methods to drive specified distance in specified time
 */

public class TWBMoves {
    // members
    public boolean reverseDir = false; // for running backwards
    
    // PieceWise member for position profiling
    final private PiecewiseFunction posVector = new PiecewiseFunction();
    
    // PieceWise member for pitch profiling
    final private PiecewiseFunction pitchVector = new PiecewiseFunction();


    // Constructor
    public TWBMoves() {

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
        pitchVector.addElement(.50,0);
        pitchVector.addElement(.60,0.4);
        pitchVector.addElement(.80,0.3);
        pitchVector.addElement(1.00,.00);
    }

    public double[] lineMove(double distance, double totalTime, double currentTime, double startingS) {
        double max_v;
        double max_a;
        double posTarget;
        double pitchTarget;
        double pitchScaler;
        int sign; // for direction

        if (reverseDir) sign = -1;
        else sign = 1;

        posTarget = sign*distance*posVector.getY(currentTime/totalTime) + startingS;

        max_v = 2.0 * distance / totalTime;
        max_a = 4.0 * max_v / totalTime;
        pitchScaler = ( max_a / 8.0) / 10.0; // convert from cm to mm
        pitchTarget = sign*pitchScaler*pitchVector.getY(currentTime/totalTime);
        if (pitchTarget > 20.0) pitchTarget = 20.0;
        else if (pitchTarget < -20.0) pitchTarget = -20.0;

        return new double[] {posTarget,pitchTarget};
    }
/*
public class MotionController {

    private double maxVelocity;
    private double maxAcceleration;

    public MotionController(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    //
     * Calculates the next state (position, velocity) given the current state and a target position,
     * respecting maximum velocity and acceleration.
     *
     * @param currentPosition The current position.
     * @param currentVelocity The current velocity.
     * @param targetPosition  The desired target position.
     * @param deltaTime       The time step for the update.
     * @return A double array containing [newPosition, newVelocity].
     //
public double[] calculateNextState(double currentPosition, double currentVelocity, double targetPosition, double deltaTime) {
    double distanceToTarget = targetPosition - currentPosition;

    // Determine direction of motion
    int direction = (distanceToTarget > 0) ? 1 : -1;

    // Calculate stopping distance required for current velocity
    double stoppingDistance = (currentVelocity * currentVelocity) / (2 * maxAcceleration);

    double acceleration = 0;

    // If approaching target and need to decelerate
    if (Math.abs(distanceToTarget) <= stoppingDistance && Math.signum(currentVelocity) == direction) {
        // Decelerate
        acceleration = -direction * maxAcceleration;
    } else {
        // Accelerate towards target, or maintain velocity
        acceleration = direction * maxAcceleration;
    }

    // Calculate new velocity
    double newVelocity = currentVelocity + acceleration * deltaTime;

    // Limit new velocity to maxVelocity
    newVelocity = Math.min(Math.abs(newVelocity), maxVelocity) * Math.signum(newVelocity);

    // If we need to decelerate and new velocity crosses zero, set to zero
    if (Math.signum(currentVelocity) != Math.signum(newVelocity) && Math.abs(distanceToTarget) <= stoppingDistance) {
        newVelocity = 0;
    }

    // Calculate new position
    double newPosition = currentPosition + newVelocity * deltaTime + 0.5 * acceleration * deltaTime * deltaTime;

    // Ensure we don't overshoot the target
    if (Math.signum(distanceToTarget) != Math.signum(targetPosition - newPosition)) {
        newPosition = targetPosition;
        newVelocity = 0;
    }

    return new double[]{newPosition, newVelocity};
}
}

    private void setPitchScaler() {
        double max_v;
        double max_a;
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
 */
}
