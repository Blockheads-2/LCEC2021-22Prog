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

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auto.cv.CoreDetection;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.positioning.MathConstHead;
import org.firstinspires.ftc.teamcode.common.positioning.MathSpline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

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

    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

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
        CoreDetection detector = new CoreDetection(telemetry);
        phoneCam.setPipeline(detector);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Error Opening Camera");
                telemetry.update();
            }
        });

        while (!opModeIsActive()) { //checks if play hasn't been pressed (in init stage)
            updateValueDecrease.update(gamepad1.a);
            updateValueIncrease.update(gamepad1.b);
            if (updateValueIncrease.is(Button.State.TAP)) {
                detector.changeHue(1);
            }

            if (updateValueDecrease.is(Button.State.TAP)) {

                detector.changeHue(-1);
            }
        }


        waitForStart();


        switch (detector.getLocation()) {
            case LEFT: {

                dispatch.spinIntake(0.2);
                dispatch.moveElevator(constants.elevatorPositionBottom-150);
                sleep(1000);
                dispatch.variableHeading(.5, 15.0, 16.5, 10);
                dispatch.spinIntake(-1,2000);
                dispatch.constantHeading(.5, 0, -4, 0.001,0,0.0003);
                dispatch.turnAbsPID(90, 1);
                dispatch.constantHeading(.5, 8, 0, 0.001,0,0.0003);
                dispatch.constantHeading(.3, 0, -40, 0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(.5, 12, 0, .001, 0, .0003);
                dispatch.spinCarousel(-1000);
                dispatch.constantHeading(0.2,8,0, 4, 0.001,0,0.0003);
                dispatch.spinCarousel(0);
                dispatch.turnAbsPID(90, 1);
                dispatch.constantHeading(0.5,0,-5,0.001,0,0.0003);
                dispatch.constantHeading(0.5,-22,0,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,-10,0.001,0,0.0003);

                /*
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionMid - 550);

                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.constantHeading(0.6,-25,20,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.variableHeading(0.6,20,1.7,1.5);
                dispatch.constantHeading(0.5,5.5,0,0.001,0,0.0003);
                dispatch.spinIntake(0);

                //out-take
                dispatch.turnAbsPID(90, 1);
                dispatch.constantHeading(0.5,5,7.25,0.001,0,0.0003);
                dispatch.spinIntake(-1,2000);

                dispatch.constantHeading(0.3,0,-24,0.001,0,0.0003);
                dispatch.constantHeading(0.5,34,0,0,0,0);

                dispatch.moveElevator(constants.elevatorPositionDown);

                dispatch.spinCarousel(-1000);
                dispatch.constantHeading(0.1,2,0,3,0,0,0);
                dispatch.spinCarousel(0);

                dispatch.constantHeading(0.5,-24,0,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,-10,0.001,0,0.0003);
                 */

                break;
            }
            case RIGHT: {
                //power on lift
                dispatch.spinIntake(0.2);
                dispatch.moveElevator(constants.elevatorPositionTop);

                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.constantHeading(0.6,-20,20,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.variableHeading(0.6,20,1.7,1.5);
                dispatch.constantHeading(0.5,8,0,0.001,0,0.0003);
                //out-take
                dispatch.turnAbsPID(90,1);
                dispatch.constantHeading(0.5,-1,10,0.001,0,0.0003);               dispatch.spinIntake(0);
                dispatch.spinIntake(0);
                dispatch.spinIntake(-1,2000);

                dispatch.constantHeading(0.3,0,-32,0.001,0,0.0003);
                dispatch.constantHeading(0.5,34,0,0,0,0);
                dispatch.constantHeading(0.3,0,-5,0.001,0,0.0003);


                dispatch.moveElevator(constants.elevatorPositionDown);

                dispatch.spinCarousel(-1000);
                dispatch.constantHeading(0.2,6,0,3,0,0,0);
                dispatch.spinCarousel(0);
                dispatch.constantHeading(0.5,-4,0,0.001,0,0.0003);
                dispatch.constantHeading(0.5,0,-10,0.001,0,0.0003);
                dispatch.constantHeading(0.5,-20,0,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,-10,0.001,0,0.0003);

                break;
            }
            case MID: {
                dispatch.spinIntake(0.2);
                dispatch.moveElevator(constants.elevatorPositionTop);

                // move to drop
                dispatch.spinIntake(0.3);
                dispatch.constantHeading(0.6,-20,20,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.variableHeading(0.6,20,1.7,1.5);
                dispatch.constantHeading(0.5,8,0,0.001,0,0.0003);
                //out-take
                dispatch.turnAbsPID(90,1);
                dispatch.constantHeading(0.5,-2.5,2.5,0.001,0,0.0003);
                dispatch.spinIntake(0);
                dispatch.spinIntake(-1,2000);

                dispatch.constantHeading(0.3,0,-32,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.5,34,0,0,0,0);
                dispatch.constantHeading(0.3,0,-5,0.001,0,0.0003);

                dispatch.spinCarousel(-1000);
                dispatch.constantHeading(0.2,6,0,3,0,0,0);
                dispatch.spinCarousel(0);
                dispatch.constantHeading(0.5,-4,0,0.001,0,0.0003);
                dispatch.constantHeading(0.5,0,-10,0.001,0,0.0003);
                dispatch.constantHeading(0.5,-20,0,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,-10,0.001,0,0.0003);

                break;
                /*
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop - 200);

                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.constantHeading(0.6,-25,20,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.variableHeading(0.6,20,1.7,1.5);
                dispatch.constantHeading(0.5,6,0,0.001,0,0.0003);
                dispatch.spinIntake(0);

                //out-take
                dispatch.constantHeading(0.5,-2,3.5,0.001,0,0.0003);
                dispatch.turnAbsPID(90,1);
                dispatch.constantHeading(0.5,4, 5, 0.001,0,0.0003);

                dispatch.spinIntake(-1,2000);

                dispatch.constantHeading(0.3,0,-24,0.001,0,0.0003);
                dispatch.constantHeading(0.5,34,0,0,0,0);

                dispatch.moveElevator(constants.elevatorPositionDown);

                dispatch.spinCarousel(-1000);
                dispatch.constantHeading(0.1,5,0,3,0,0,0);
                dispatch.spinCarousel(0);

                dispatch.constantHeading(0.5,-24,0,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,-10,0.001,0,0.0003);
                 */

            }
        }
        phoneCam.stopStreaming();
        telemetry.update();
    }
}