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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auto.cv.BlueDetection;
import org.firstinspires.ftc.teamcode.auto.cv.RedDetection;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
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

@Autonomous(name="Red Carousel", group="Routes")

//@Disabled
public class RedCarousel extends LinearOpMode {

    /* Declare OpMode members. */
    OpenCvCamera phoneCam;
    HardwareDrive         robot   = new HardwareDrive();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    Constants constants = new Constants();
    MathSpline mathSpline = new MathSpline();
    MathConstHead mathConstHead = new MathConstHead();

    double degreeConversion = constants.degree;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: TETRIX Motor Encoder
    static final double     MAX_VELOCITY_DT         = 2700;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   =  (96.0/25.4);     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    AutoHub dispatch;

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

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


        waitForStart();

        switch (detector.getLocation()) {
            case LEFT: {

                //power on lift
                dispatch.moveElevator(constants.elevatorPositionBottom);

                // move to drop
                dispatch.spinIntake(0.2);
                dispatch.constantHeading(0.5,10,0,0.5,0.001,0,0.0003);
                dispatch.variableHeading(0.6,10,20,1.5);

                //out-take
                dispatch.constantHeading(0.5,0,1,0.4,0.001,0,0.0003);
                dispatch.spinIntake(-0.75,2000);
                dispatch.constantHeading(0.5,0,-1,0.4,0.001,0,0.0003);


                //to carousel
                dispatch.variableHeading(0.5,-20,-10,1.2);

                //move to carousel
                dispatch.constantHeading(0.4,0,-20,1.5,0.001,0,0.0003);
                dispatch.constantHeading(0.4,10,0,1.5,0.001,0,0.0003);

                //spin
                dispatch.spinCarousel(-1600);
                //lift down
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.1,2,0,4.1,0.001,0,0.0003);

                /*
                //Stop Elevator
                if (robot.digitalTouch.getState() == false) {
                    //Stop
                    robot.lifter.setPower(0);

                    //Reset
                    robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                 */

                dispatch.spinCarousel(0);

                //move to park
                dispatch.constantHeading(0.5, -22, 0, 2,0.001,0,0.0003);
                dispatch.constantHeading(0.5, 0, -5, 2,0.001,0,0.0003);


                telemetry.addLine("Path: Left");
                break;
            }
            case MID: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionMid);

                // move to drop
                dispatch.constantHeading(0.4,-12,0,1,0.001,0,0.0003);
                robot.spin.setPower(0.2);
                dispatch.constantHeading(0.8, 0, 18, 1,0.001,0,0.0003);
                dispatch.variableHeading(0.6,19,14.825,1.5);
                dispatch.turnPID(15,2);
                dispatch.constantHeading(0.5,0,2,0.25,0.001,0,0.0003);

                //out-take
                dispatch.spinIntake(-1,2000);



                //to carousel
                dispatch.constantHeading(0.5,0,-5,0.5,0.001,0,0.0003);

                dispatch.constantHeading(0.2,0,-3,0.5,0.001,0,0.0003);
                dispatch.constantHeading(0.4, 24, -24, 3.0,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,-5,0.5,0.001,0,0.0003);

                //move to carousel
                dispatch.constantHeading(0.7,14,0,1.5,0.001,0,0.0003);

                //spin
                dispatch.spinCarousel(-1600);
                // lift down
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.1,2,0,4.1,0.001,0,0.0003);

                /*

            //Stop Elevator
                if (robot.digitalTouch.getState() == false) {
                    //Stop
                    robot.lifter.setPower(0);

                    //Reset
                    robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                 */

                dispatch.spinCarousel(0);

                //move to park
                dispatch.constantHeading(0.5, -22, 0, 2,0.001,0,0.0003);
                dispatch.constantHeading(0.5,0,-5,1,0.001,0,0.0003);

                telemetry.addLine("Path: Mid");
                break;
            }
            case RIGHT:{

                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop);

                // move to drop
                dispatch.constantHeading(0.4,-8,0,1,0.001,0,0.0003);
                robot.spin.setPower(0.2);
                dispatch.constantHeading(0.8, 0, 16, 1,0.001,0,0.0003);
                dispatch.variableHeading(0.6,19,14.825,1.5);

                //out-take
                dispatch.spinCarousel(-1,2000);



                //spline to carousel
                dispatch.constantHeading(0.5,-5,0,0.5,0.001,0,0.0003);
                dispatch.turnPID(10,1);
                dispatch.constantHeading(0.2,0,-3,0.5,0.001,0,0.0003);
                dispatch.constantHeading(0.6, 26, -26, 2.0,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,-5,0.5,0.001,0,0.0003);

                //move to carousel
                dispatch.constantHeading(0.7,20,0,1.5,0.001,0,0.0003);

                //spin
                dispatch.spinCarousel(-1600);
                //lift down
                dispatch.moveElevator(constants.elevatorPositionDown);

                dispatch.constantHeading(0.1,2,0,4.1,0.001,0,0.0003);

                /*
                //Stop Elevator
                if (robot.digitalTouch.getState() == false) {
                    //Stop
                    robot.lifter.setPower(0);

                    //Reset
                    robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                 */

                dispatch.spinCarousel(0);

                //move to park
                dispatch.constantHeading(0.5, -21, 0, 2,0.001,0,0.0003);

                telemetry.addLine("Path: Right");
                break;
            }
        }

        phoneCam.stopStreaming();
        telemetry.update();
    }
}