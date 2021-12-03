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

package org.firstinspires.ftc.teamcode.auto.routes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.auto.cv.RedDetection;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Red Carousel Auto", group="Routes")

//@Disabled
public class RedCarouselAuto extends LinearOpMode {

    /* Declare OpMode members. */
    OpenCvCamera phoneCam;
    HardwareDrive         robot   = new HardwareDrive();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    Constants constants = new Constants();

    double degreeConversion = constants.degree;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: TETRIX Motor Encoder
    static final double     MAX_VELOCITY_DT         = 2700;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   =  (96.0/25.4);     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders and Camera");
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        RedDetection detector = new RedDetection(telemetry);
        phoneCam.setPipeline(detector);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Error Opening Camera");
                telemetry.update();
            }
        });

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        switch (detector.getLocation()) {
            case LEFT: {
                //...
                telemetry.addLine("Path: Left");
                break;
            }
            case MID: {
                //...
                telemetry.addLine("Path: Mid");
                break;
            }
            case RIGHT:{

                //power on lift
                robot.lifter.setTargetPosition(constants.elevatorPositionTop);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(1);

                // spline to drop
                variableHeading(0.3,12,5,2);

                //out-take
                robot.spin.setPower(-0.3);
                sleep(1000);
                robot.spin.setPower(0);

                //lift down
                robot.lifter.setTargetPosition(constants.elevatorPositionDown);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(1);

                //spline to carousel
                variableHeading(0.5,12,12,3);

                //spin
                robot.duckWheel.setPower(0.7);
                sleep(2300);
                robot.duckWheel.setPower(0);

                //move to park
                variableHeading(0.5,10,10,20);

                telemetry.addLine("Path: Right");
                break;
            }
        }

        phoneCam.stopStreaming();
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