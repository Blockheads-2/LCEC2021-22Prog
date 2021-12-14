package org.firstinspires.ftc.teamcode.common.pid;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MovePIDController {
        private double currentTime, previousTime, deltaTime;
        private double integral = 0;
        private double currentError, previousError, deltaError, derivative;
        private final double kP;
        private final double kI;
        private final double kD;
        private final double targetValue;
        private double previousOutput;

        public MovePIDController (double kP, double kI, double kD, double targetValue) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.targetValue = targetValue;
            this.currentTime = getTimeSeconds();
        }

        public double getCorrectedOutput(double processValue) {
            previousTime = currentTime;
            currentTime = getTimeSeconds();
            deltaTime = currentTime - previousTime;
            currentError = processValue - targetValue;

            if (deltaTime > 0) {
                deltaError = currentError - previousError;

                derivative =  deltaError/deltaTime;
                integral += currentError * deltaTime;

                double output = -(currentError * kP + integral * kI + derivative * kD);

                previousError = currentError;
                previousOutput = output;

                return output;
            }

            return previousOutput;
        }

        public void reset() {
            currentTime = getTimeSeconds();
            previousError = currentTime;
            deltaTime = 0;
            previousError = currentError;
            derivative = 0;
            integral = 0;
        }

        public double getError() {
            return currentError;
        }

        public double getTargetValue() {
            return targetValue;
        }

        public double getkP() {
            return kP;
        }

        private double getTimeSeconds() {
            return System.currentTimeMillis() / 1000.0;
        }
}