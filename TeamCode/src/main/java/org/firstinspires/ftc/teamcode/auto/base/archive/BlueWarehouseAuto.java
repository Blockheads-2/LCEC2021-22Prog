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

package org.firstinspires.ftc.teamcode.auto.base.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auto.cv.BlueWarehouseDetection;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.positioning.MathConstHead;
import org.firstinspires.ftc.teamcode.common.positioning.MathSpline;
import org.firstinspires.ftc.teamcode.common.pid.TurnPIDController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Blue Warehouse Auto", group="Routes")

@Disabled
public class BlueWarehouseAuto extends LinearOpMode {

    /* Declare OpMode members. */
    OpenCvCamera phoneCam;
    HardwareDrive         robot   = new HardwareDrive();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    Constants constants = new Constants();
    MathSpline mathSpline = new MathSpline();
    MathConstHead mathConstHead = new MathConstHead();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
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
        BlueWarehouseDetection detector = new BlueWarehouseDetection(telemetry);
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
                //Hit barrier
                constantHeading(0.5,0,13,1);
                constantHeading(0.5,-12,0,1);

                robot.spin.setPower(0.2);

                //lift arm
                robot.lifter.setTargetPosition(constants.elevatorPositionBottom - 400);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(0.9);

                //Move along barrier and toward wobble
                constantHeading(0.5,-5,24,1.3);
                constantHeading(0.5,2,0,0.5);
                turnAbsPID(-90);
                robot.spin.setPower(0);
                constantHeading(0.5,0,-2,0.5);
                constantHeading(0.675,2.8,2.8,1);
                constantHeading(0.3,0,1.3,0.5);
                turnPID(20);

                //outtake
                robot.spin.setPower(-0.7);
                sleep(3800);
                robot.spin.setPower(0);

                //park
                constantHeading(0.8,0,-6,1);
                constantHeading(0.5,0,2,0.75);
                turnPID(90);
                robot.lifter.setTargetPosition(constants.elevatorPositionDown);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(0.9);
                constantHeading(0.5,-3,0,0.5);
                constantHeading(0.5,0,-44,3);
                constantHeading(0.5,0,3,0.7);
                turnPID(100);
                constantHeading(0.5,-3,0,0.7);
                robot.spin.setPower(1);
                constantHeading(0.8,0,33,1.2);
                constantHeading(0.2,0,2,0.3);
                constantHeading(0.25,3,1,0.75);
                sleep(1000);

                telemetry.addLine("Path: Left");
                break;
            }
            case RIGHT: {
                robot.lifter.setTargetPosition(constants.elevatorPositionTop);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(0.9);
                robot.spin.setPower(0.3);


                //move to wobble
                variableHeading(0.5,6.25,17.1,1.5);
                constantHeading(0.5,0,1.5,0.4);

                //outtake
                robot.spin.setPower(-1);
                sleep(2300);

                //park
                constantHeading(0.5,0,-1.5,0.4);
                variableHeading(0.5,6.25,-17.1,1.5);
                robot.lifter.setTargetPosition(constants.elevatorPositionDown);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(0.9);
                constantHeading(0.5,0,3,0.7);
                turnPID(100);
                constantHeading(0.5,-4,0,0.7);
                robot.spin.setPower(1);
                constantHeading(0.8,0,33,1.2);
                constantHeading(0.2,0,1.5,0.3);
                constantHeading(0.25,3,2,0.75);
                sleep(1000);

                telemetry.addLine("Path: Right");
                break;
            }
            case MID: {
                //lift arm
                robot.lifter.setTargetPosition(constants.elevatorPositionMid-400);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(0.9);
                robot.spin.setPower(0.3);


                //move to wobble
                variableHeading(0.5,6.1,16.75,1.5);
                constantHeading(0.5,0,3,0.4);

                //outtake
                robot.spin.setPower(-1);
                sleep(2300);

                //park
                constantHeading(0.5,0,-3,0.4);
                variableHeading(0.5,6.25,-17,1.5);
                robot.lifter.setTargetPosition(constants.elevatorPositionDown);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(0.9);
                constantHeading(0.5,0,3,0.7);
                turnPID(100);
                constantHeading(0.5,-4,0,0.7);
                robot.spin.setPower(1);
                constantHeading(0.8,-2,17,0.7);
                constantHeading(0.8,-2,17,0.7);
                constantHeading(0.2,0,1.5,0.3);
                constantHeading(0.35,3.5,2,0.75);
                sleep(500);

                telemetry.addLine("Path: Mid");
                break;
            }
        }

        phoneCam.stopStreaming();
        telemetry.update();
    }

    //Functions for Moving
    public void variableHeading(double speed, double xPose, double yPose, double timeoutS) {
        int FleftEncoderTarget;
        int FrightEncoderTarget;
        int BleftEncoderTarget;
        int BrightEncoderTarget;

        double leftDistance;
        double rightDistance;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            speed = speed * MAX_VELOCITY_DT;

            mathSpline.setFinalPose(xPose,yPose);

            leftDistance = mathSpline.returnLDistance() * COUNTS_PER_INCH;
            rightDistance = mathSpline.returnRDistance() * COUNTS_PER_INCH;

            if ((yPose >= 0 && xPose < 0) || (yPose < 0 && xPose >= 0)){
                FleftEncoderTarget = robot.lf.getCurrentPosition() - (int) leftDistance;
                FrightEncoderTarget = robot.rf.getCurrentPosition() - (int) rightDistance;
                BleftEncoderTarget = robot.lb.getCurrentPosition() - (int) leftDistance;
                BrightEncoderTarget = robot.rb.getCurrentPosition() - (int) rightDistance;
            }
            else {
                FleftEncoderTarget = robot.lf.getCurrentPosition() + (int) leftDistance;
                FrightEncoderTarget = robot.rf.getCurrentPosition() + (int) rightDistance;
                BleftEncoderTarget = robot.lb.getCurrentPosition() + (int) leftDistance;
                BrightEncoderTarget = robot.rb.getCurrentPosition() + (int) rightDistance;
            }


            robot.lf.setTargetPosition(FleftEncoderTarget);
            robot.lb.setTargetPosition(BleftEncoderTarget);
            robot.rf.setTargetPosition(FrightEncoderTarget);
            robot.rb.setTargetPosition(BrightEncoderTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.lf.setVelocity(speed * mathSpline.returnLPower());
            robot.rf.setVelocity(speed * mathSpline.returnRPower());
            robot.lb.setVelocity(speed * mathSpline.returnLPower());
            robot.rb.setVelocity(speed * mathSpline.returnRPower());

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.lf.isBusy() && robot.rf.isBusy()
                    && robot.lb.isBusy() && robot.rb.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Right Distance", mathSpline.returnRDistance());
                telemetry.addData("Left Distance", mathSpline.returnLDistance());
                telemetry.update();
            }

            // Stop all motion;
            robot.lf.setPower(-0.25);
            robot.rf.setPower(-0.25);
            robot.lb.setPower(-0.25);
            robot.rb.setPower(-0.25);

            sleep(100);

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
    public void constantHeading(double speed, double xPose, double yPose, double timeoutS) {
        mathConstHead.setFinalPose(xPose,yPose);

        double distance = mathConstHead.returnDistance();
        double radianAngle = mathConstHead.returnAngle();

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        double ratioAddPose = Math.cos(radianAngle) + Math.sin(radianAngle);
        double ratioSubPose = Math.cos(radianAngle) - Math.sin(radianAngle);
        int addPose = (int) (ratioAddPose * COUNTS_PER_INCH * distance);
        int subtractPose = (int) (ratioSubPose * COUNTS_PER_INCH * distance);

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


            robot.lf.setVelocity(speed * constants.maxVelocityDT * ratioAddPose);
            robot.rf.setVelocity(speed * constants.maxVelocityDT * ratioSubPose);
            robot.lb.setVelocity(speed * constants.maxVelocityDT * ratioSubPose * 0.9);
            robot.rb.setVelocity(speed * constants.maxVelocityDT * ratioAddPose * 0.9);

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

    //Turn
    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }
    public double getAngle() {
        // Get current orientation
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }
    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.lf.setPower(-motorPower);
            robot.rf.setPower(motorPower);
            robot.lb.setPower(-motorPower);
            robot.rb.setPower(motorPower);

            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);

    }
    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }
    public void turnPID(double degrees) {
        mathPID(-degrees + getAbsoluteAngle());
    }
    public void turnAbsPID(double absDegrees){
        mathPID(-absDegrees);
    }
    void mathPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.lf.setPower(-motorPower);
            robot.rf.setPower(motorPower);
            robot.lb.setPower(-motorPower);
            robot.rb.setPower(motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
    }
}