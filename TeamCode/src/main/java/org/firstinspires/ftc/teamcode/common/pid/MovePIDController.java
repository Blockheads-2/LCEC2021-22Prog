package org.firstinspires.ftc.teamcode.common.pid;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MovePIDController{

    private double kP, kI, kD;
    private ElapsedTime timer = new ElapsedTime();
    private double targetDistance = 0;
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;

    public MovePIDController(double target, double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        targetDistance = target;
    }

    public double update(double currentDistance) {
        // TODO: make sure angles are within bounds and are in same format (e.g., 0 <= | angle | <= 180)
        //   and ensure direction is correct

        // P
        double error = targetDistance - currentDistance;

        // I
        accumulatedError *= Math.signum(error);
        accumulatedError += error;
        if (Math.abs(error) < 2) {
            accumulatedError = 0;
        }

        // D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastSlope = slope;
        lastError = error;
        lastTime = timer.milliseconds();

        double motorPower = 0.9 * Math.tanh(kP * error + kI * accumulatedError + kD * slope);
        return motorPower;
    }

    public double getLastSlope() {
        return lastSlope;
    }

}