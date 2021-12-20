/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto.base.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.common.Constants;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Disabled
@Autonomous(name="Encoder Auto Drive", group="Robot Base Drive")

//Start of Class
public class EncoderAutoDrive extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    Constants constants = new Constants();

    int MAX_VELOCITY_DT = 2700;


    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = (96.0 / 25.4);     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Test Paths Start

        //...
        constantHeading(0.5,10,-10,3);

        //End of Path
        telemetry.update();
    }


    //Functions for Moving
    public void variableHeading(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        double fractionBetweenLeftAndRight;
        double reducedSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            speed = speed * MAX_VELOCITY_DT;

            if (Math.abs(leftInches) > Math.abs(rightInches))
                fractionBetweenLeftAndRight = rightInches / leftInches;
            else if (Math.abs(leftInches) < Math.abs(rightInches))
                fractionBetweenLeftAndRight = leftInches / rightInches;
            else
                fractionBetweenLeftAndRight = 1;

            reducedSpeed = speed * fractionBetweenLeftAndRight;

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.lf.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rf.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.lb.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rb.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            if (Math.abs(leftInches) > Math.abs(rightInches)) {
                robot.lf.setVelocity(speed);
                robot.rf.setVelocity(reducedSpeed);
                robot.lb.setVelocity(speed);
                robot.rb.setVelocity(reducedSpeed);
            } else if (Math.abs(leftInches) == Math.abs(rightInches)) {
                robot.lf.setVelocity(speed);
                robot.lb.setVelocity(speed);
                robot.rf.setVelocity(speed);
                robot.rb.setVelocity(speed);
            } else {
                robot.lf.setVelocity(reducedSpeed);
                robot.rf.setVelocity(speed);
                robot.lb.setVelocity(reducedSpeed);
                robot.rb.setVelocity(speed);
            }

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.lf.isBusy() && robot.rf.isBusy()
                    && robot.lb.isBusy() && robot.rb.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Left Velocity: ", robot.lf.getVelocity());
                telemetry.addData("Right Velocity: ", robot.rf.getVelocity());
                telemetry.update();
            }

            // Stop all motion;
            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void constantHeading(double speed, double distance, double angle, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        double radianAngle = Math.toRadians(angle);

        int addPose = (int) (distance * (Math.sin(radianAngle) + Math.cos(radianAngle)) * COUNTS_PER_INCH);
        int subtractPose = (int) (distance * (Math.cos(radianAngle) - Math.sin(radianAngle)) * COUNTS_PER_INCH);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.lf.getCurrentPosition() + addPose;
            newRightFrontTarget = robot.rf.getCurrentPosition() + subtractPose;
            newLeftBackTarget = robot.lb.getCurrentPosition() + subtractPose;
            newRightBackTarget = robot.rb.getCurrentPosition() + addPose;

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            robot.lf.setVelocity(speed * constants.maxVelocityDT);
            robot.rf.setVelocity(speed * constants.maxVelocityDT);
            robot.lb.setVelocity(speed * constants.maxVelocityDT * 0.9);
            robot.rb.setVelocity(speed * constants.maxVelocityDT * 0.9);

            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                // Display it for the driver.
                telemetry.addData("Left Velocity: ", robot.lb.getVelocity());
                telemetry.addData("Right Velocity: ", robot.rb.getVelocity());
                telemetry.update();
            }

            // Stop all motion;
            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void TurnLeft(double speed, double degrees, double timeoutS) {

        double inches = degrees * constants.degree;

        //Initial Target Position
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        //Ramp Down
        int halfLeftFrontTarget;
        int halfRightFrontTarget;
        int halfLeftBackTarget;
        int halfRightBackTarget;

        //Current Motor Positions
        int currentLeftFront;
        int currentRightFront;
        int currentLeftBack;
        int currentRightBack;

        speed = COUNTS_PER_MOTOR_REV * speed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.lf.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rf.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.lb.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rb.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            halfLeftFrontTarget = robot.lf.getCurrentPosition() + (int) (-inches / 2 * COUNTS_PER_INCH);
            halfRightFrontTarget = robot.rf.getCurrentPosition() + (int) (inches / 2 * COUNTS_PER_INCH);
            halfLeftBackTarget = robot.lb.getCurrentPosition() + (int) (-inches / 2 * COUNTS_PER_INCH);
            halfRightBackTarget = robot.rb.getCurrentPosition() + (int) (inches / 2 * COUNTS_PER_INCH);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.lf.setVelocity(Math.abs(speed));
            robot.rf.setVelocity(Math.abs(speed));
            robot.lb.setVelocity(Math.abs(speed));
            robot.rb.setVelocity(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.lf.isBusy() && robot.rf.isBusy()) && robot.lb.isBusy() && robot.rb.isBusy()) {

                currentLeftFront = robot.lf.getCurrentPosition();
                currentRightFront = robot.rf.getCurrentPosition();
                currentLeftBack = robot.lb.getCurrentPosition();
                currentRightBack = robot.rb.getCurrentPosition();

                if (currentLeftFront >= halfLeftFrontTarget && currentRightFront >= halfRightFrontTarget
                        && currentLeftBack >= halfLeftBackTarget && currentRightBack >= halfRightBackTarget) {
                    speed = speed * 0.95;
                    if (speed <= 0.2) {
                        speed = 0.2;
                    }
                }
                robot.lf.setVelocity(Math.abs(speed));
                robot.lb.setVelocity(Math.abs(speed));
                robot.rf.setVelocity(Math.abs(speed));
                robot.rb.setVelocity(Math.abs(speed));
            }

            // Stop all motion;
            robot.lf.setVelocity(0);
            robot.rf.setVelocity(0);
            robot.lb.setVelocity(0);
            robot.rb.setVelocity(0);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }
    public void TurnRight(double speed, double degrees, double timeoutS) {

        double inches = degrees * constants.degree;

        //Initial Target Position
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        //Ramp Down
        int halfLeftFrontTarget;
        int halfRightFrontTarget;
        int halfLeftBackTarget;
        int halfRightBackTarget;

        //Current Motor Positions
        int currentLeftFront;
        int currentRightFront;
        int currentLeftBack;
        int currentRightBack;

        speed = COUNTS_PER_MOTOR_REV * speed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.lf.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rf.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.lb.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rb.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);

            halfLeftFrontTarget = robot.lf.getCurrentPosition() + (int) (inches / 2 * COUNTS_PER_INCH);
            halfRightFrontTarget = robot.rf.getCurrentPosition() + (int) -(inches / 2 * COUNTS_PER_INCH);
            halfLeftBackTarget = robot.lb.getCurrentPosition() + (int) (inches / 2 * COUNTS_PER_INCH);
            halfRightBackTarget = robot.rb.getCurrentPosition() + (int) (-inches / 2 * COUNTS_PER_INCH);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.lf.setVelocity(Math.abs(speed));
            robot.rf.setVelocity(Math.abs(speed));
            robot.lb.setVelocity(Math.abs(speed));
            robot.rb.setVelocity(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.lf.isBusy() && robot.rf.isBusy()) && robot.lb.isBusy() && robot.rb.isBusy()) {

                currentLeftFront = robot.lf.getCurrentPosition();
                currentRightFront = robot.rf.getCurrentPosition();
                currentLeftBack = robot.lb.getCurrentPosition();
                currentRightBack = robot.rb.getCurrentPosition();

                if (currentLeftFront >= halfLeftFrontTarget && currentRightFront >= halfRightFrontTarget
                        && currentLeftBack >= halfLeftBackTarget && currentRightBack >= halfRightBackTarget) {
                    speed = speed * 0.95;
                    if (speed <= 0.2) {
                        speed = 0.2;
                    }
                }
                robot.lf.setVelocity(Math.abs(speed));
                robot.lb.setVelocity(Math.abs(speed));
                robot.rf.setVelocity(Math.abs(speed));
                robot.rb.setVelocity(Math.abs(speed));
            }

            // Stop all motion;
            robot.lf.setVelocity(0);
            robot.rf.setVelocity(0);
            robot.lb.setVelocity(0);
            robot.rb.setVelocity(0);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move
        }
    }
}