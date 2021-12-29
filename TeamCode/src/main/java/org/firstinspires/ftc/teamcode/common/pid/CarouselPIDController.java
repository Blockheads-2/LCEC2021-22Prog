package org.firstinspires.ftc.teamcode.common.pid;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.Constants;

public class CarouselPIDController {
    private double kP, kI, kD;
    private ElapsedTime timer = new ElapsedTime();
    private double targetVelocity;
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;

    Constants constants = new Constants();

    public CarouselPIDController(double target, double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        targetVelocity = target;
    }

    public double update(double currentVelocity, boolean clockwise) {
        // P
        double error = targetVelocity - currentVelocity;

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

        double motorVelocity = kP * error + kI * accumulatedError + kD * slope;
        if (clockwise)
            motorVelocity += constants.spinClockwise;
        else
            motorVelocity += constants.spinCounterClockwise;
        return motorVelocity;
    }
}
