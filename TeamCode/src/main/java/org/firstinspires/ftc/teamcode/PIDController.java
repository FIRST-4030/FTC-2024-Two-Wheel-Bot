/*
PID Controller Class, with original source from ChatGPT
Enhancements from SrAmo (utilize rate if provided, getIntegral, Integral reset)
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class PIDController {
    private double kp;  // Proportional gain
    private double ki;  // Integral gain
    private double kd;  // Derivative gain

    private double setpoint;  // Desired value
    private double integral;  // Integral sum
    private double previousError;  // Previous error value
    private long lastTime;  // Last timestamp
    
    /* Constructor for PIDController */
    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.setpoint = 0.0;
        this.integral = 0.0;
        this.previousError = 0.0;
        this.lastTime = System.currentTimeMillis();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    //    Compute method with single feedback parameter //
    public double compute(double processVariable) {
        long now = System.currentTimeMillis();
        double timeChange = (now - lastTime) / 1000.0;  // Convert milliseconds to seconds

        double error = setpoint - processVariable;
        integral += error * timeChange; // accumulate the error over time
        double derivative = (error - previousError) / timeChange;

        double output = kp * error + ki * integral + kd * derivative;

        previousError = error;
        lastTime = now;

        return output;
    }
    //

    /* Compute method with double parameters (feedback=processVariable and rate=derivative) 
       IMU ususally returns angular rates, which can be used rather than deriving the derivative */
    public double compute(double processVariable, double derivative) {
        double output;
        long now = System.currentTimeMillis();
        double timeChange = (now - lastTime) / 1000.0;  // Convert milliseconds to seconds
        
        double error = setpoint - processVariable;
        
        // windup control - don't add to error if large
        if ((error < 35.0) && (error >-35.0)) { // normal range
            integral += error * timeChange; // accumulate the error over time
            output = kp * error + ki * integral - kd * derivative;
        } else if (error >= 10) {
            output = 1.0;
        } else {
            output = -1.0;
        }

        // NOTE: SPECIAL PITCH PID METHOD 
        // kp is squared (0.0222 IS BECAUSE ITS USING DEGREES)  AND
        // step function proportion  AND
        // BLACK = 0.04
        // DERIVATIVE SIGN IS REVERESED IN THIS METHOD
        // FOR BLACK:
        //double output = kp * 0.0222 * error * error * Math.signum(error) + 0.015*Math.signum(error) + ki * integral - kd * derivative;
        //double output = kp * error + ki * integral - kd * derivative;
        // FOR BLUE:
        //double output = kp * Math.pow(Math.abs(error),0.5)*Math.signum(error) + ki * integral - kd * derivative;

        previousError = error; 
        lastTime = now;
        
        Range.clip(output,-1.0,1.0);  // this is the max range for setPower

        return output;
    }

    public void reset() {
        this.integral = 0.0;
        this.previousError = 0.0;
        this.lastTime = System.currentTimeMillis();
    }

    // Getters and setters for gains
    public double getKp() {
        return kp;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public double getKi() {
        return ki;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public double getKd() {
        return kd;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public double getIntegral() {
        return integral;
    }
}
