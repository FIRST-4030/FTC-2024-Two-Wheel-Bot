package org.firstinspires.ftc.teamcode;

// The TWB Motion Control object encapsulates a degree of freedom term,
// such as a robot position (s) or a robot arm angle,
// and provides methods to control and limit that degree of freedom.
// Limits such as position limits, velocity limits, rate of change limits.
// Rate of change limits help to control accelerations, and provide smooth motion.
public class TWBMotionControl {
    // Members

    private double currentPos; // current position (distance)
    private double targetPos; // target position
    private double posMax; // maximum position
    private double posMin; // minimum position
    private double maxVelocity; // maximum dist/sec allowed (always positive, work either direction)
    private double currentRate; // current rate (velocity)
    private double targetRate; // target rate
    private double maxAcceleration; // maximum acceleration dist/sec^2 allowed, always positive

    // Constructors (to initialize the object
    public TWBMotionControl(double initPos, double maxPos, double minPos, double maxRate, double initRate, double maxA) {
        this.currentPos = initPos;
        this.targetPos = initPos;
        this.posMax = maxPos;
        this.posMin = minPos;
        this.maxVelocity = Math.abs(maxRate);
        this.currentRate = initRate;
        this.targetRate = initRate;
        this.maxAcceleration = Math.abs(maxA);
    }

    /**
     * Calculates the next state (position, velocity) given the current state and a target position,
     * respecting maximum velocity and acceleration.
     *
     * @param currentPosition The current position.
     * @param currentVelocity The current velocity.
     * @param targetPosition  The desired target position.
     * @param deltaTime       The time step for the update.
     * @return A double array containing [newPosition, newVelocity].
     */
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
    public void setTargetPos(double newTargetPos) { targetPos = newTargetPos; }

    public void setCurrentPos(double pos) { currentPos = pos; }

    public double getCurrentPos() {  return currentPos;   }

    public double getTargetPos() { return targetPos; }

    // NOT WORKING: Update Position using velocity limit and min/max position limit
    // (expected to be called continuously within loop)
    // Call either updatePosition or updateVelocity, once per loop.
    public void updatePosition(double deltaTime, boolean limitPos) {
        double deltaPos = targetPos-currentPos; // position change being asked for
        double deltaPosMax = maxVelocity * deltaTime; // convert rate limit to delta position based on loop time
        // apply velocity (rate) limit
        if (Math.abs(deltaPos) > deltaPosMax) deltaPos = Math.signum(deltaPos) * deltaPosMax;

        double deltaVelo = deltaPos/deltaTime-currentRate; // difference in rate between this and prior time
        double deltaVeloMax = maxAcceleration * deltaTime; // convert acceleration to velocity based on loop time
        if (Math.abs(deltaVelo) > deltaVeloMax) {
            deltaVelo = Math.signum(deltaVelo) * deltaVeloMax;
            deltaPos = deltaVelo*deltaTime;
        }

        targetPos += deltaPos; // update current position

        // apply acceleration limit, which could further reduce the change
        setTargetRate(deltaPos/deltaTime); // set the target rate (new rate)
        setCurrentRate(deltaPos/deltaTime); // save the velocity for the next time through the loop

        // check limits
        if (limitPos) {
            if (currentPos > posMax) targetPos = posMax;
            else if (currentPos < posMin) targetPos = posMin;
        }
    }

    public void setTargetRate(double newTargetRate) { targetRate = newTargetRate;}

    public void setCurrentRate(double newCurrentRate) {currentRate = newCurrentRate;}

    public double getTargetRate() { return targetRate; }

}
