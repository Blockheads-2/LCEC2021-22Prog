package org.firstinspires.ftc.teamcode.auto.routes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.cv.BlueWarehouseDetection;
import org.firstinspires.ftc.teamcode.auto.cv.CoreDetection;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.common.Constants;

@Autonomous(name = "Blue Warehouse", group = "Routes")
public class BlueWarehouse extends LinearOpMode {
    OpenCvCamera phoneCam;
    AutoHub dispatch;
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        BlueWarehouseDetection detector = new BlueWarehouseDetection(telemetry);
        phoneCam.setPipeline(detector);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error Opening Camera");
                telemetry.update();
            }
        });


        waitForStart();
        switch (detector.getLocation()) {
            case LEFT: {
                dispatch.moveElevator(constants.elevatorPositionBottom);
                sleep(750);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.5,7,15,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 2000);
                dispatch.constantHeading(0.5,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(0.3,0,-18,0.001,0,0.0003);
                dispatch.turnAbsPID(-90, 1);
                dispatch.constantHeading(0.2,-5,0,0.001,0,0.0003);
                dispatch.constantHeading(0.5, 0, 32, 0.001,0,0.0003);

                /*
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionBottom);

                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.constantHeading(0.6,30,20,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,0,21,0.001,0,0.0003);
                dispatch.variableHeading(0.6,-10,1.7,1.5);
                dispatch.constantHeading(0.5,-5.5,0,0.001,0,0.0003);
                dispatch.spinIntake(0);
                dispatch.constantHeading(0.5,0,.8,0.001,0,0.0003);


                //out-take
                dispatch.constantHeading(0.5,-2,7,0.001,0,0.0003);
                dispatch.spinIntake(-1,2000);

                dispatch.constantHeading(0.5,0,-5,0.001,0,0.0003);

                dispatch.moveElevator(constants.elevatorPositionDown);
                //move to carousel -- start ---
                dispatch.turnAbsPID(270,0.5);
                dispatch.constantHeading(0.3,0,-35,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,1,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,4,0,0.001,0,0.0003);
                dispatch.constantHeading(0.45,10,-30,0.001,0,0.0003);

                dispatch.spinCarousel(1400);
                dispatch.constantHeading(0.2,0,-6,2.5,0.001,0,0.0003);
                dispatch.spinCarousel(0);
                dispatch.constantHeading(0.5,5,0,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,24,0.001,0,0.0003);
                dispatch.turnAbsPID(-90,1);
                */
                break;

            }
            case RIGHT: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop);
                sleep(750);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.5,7,15,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 2000);
                dispatch.constantHeading(0.5,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(0.3,0,-18,0.001,0,0.0003);
                dispatch.turnAbsPID(-90, 1);
                dispatch.constantHeading(0.2,-5,0,0.001,0,0.0003);
                dispatch.constantHeading(0.5, 0, 32, 0.001,0,0.0003);

//                dispatch.moveElevator(constants.elevatorPositionDown);
//                //move to carousel -- start ---
//                dispatch.turnAbsPID(270, 0.5);
//                dispatch.constantHeading(0.3, 0, -35, 0.001, 0, 0.0003);
//                dispatch.constantHeading(0.3, 0, 1, 0.001, 0, 0.0003);
//                dispatch.turnAbsPID(0, 1);
//                dispatch.constantHeading(0.5, 4, 0, 0.001, 0, 0.0003);
//                dispatch.constantHeading(0.45, 10, -30, 0.001, 0, 0.0003);
//
//                dispatch.spinCarousel(1400);
//                dispatch.constantHeading(0.2, 0, -6, 2.5, 0.001, 0, 0.0003);
//                dispatch.spinCarousel(0);
//                dispatch.constantHeading(0.5, 5, 0, 0.001, 0, 0.0003);
//                dispatch.constantHeading(0.3, 0, 24, 0.001, 0, 0.0003);
//                dispatch.turnAbsPID(-90, 1);


                break;
            }
            case MID: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop - 200);
                sleep(750);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.5,7,12,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 2000);
                dispatch.constantHeading(0.5,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(0.3,0,-18,0.001,0,0.0003);
                dispatch.turnAbsPID(-90, 1);
                dispatch.constantHeading(0.2,-5,0,0.001,0,0.0003);
                dispatch.constantHeading(0.5, 0, 32, 0.001,0,0.0003);

                /*
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.constantHeading(0.6, 30, 20, 0.001, 0, 0.0003);
                dispatch.turnAbsPID(0, 1);
                dispatch.constantHeading(0.5, 0, 21, 0.001, 0, 0.0003);
                dispatch.variableHeading(0.6, -10, 1.7, 1.5);
                dispatch.constantHeading(0.5, -5.5, 0, 0.001, 0, 0.0003);
                dispatch.spinIntake(0);

                dispatch.turnAbsPID(270, 1);
                dispatch.constantHeading(.5, -10.5, 2, .001, 0, .0003);
                sleep(500);

                //out-take
                dispatch.spinIntake(-1, 2000);

                dispatch.constantHeading(0.5, 0, -5, 0.001, 0, 0.0003);

                dispatch.moveElevator(constants.elevatorPositionDown);
                //move to carousel -- start ---
                dispatch.turnAbsPID(270, 0.5);
                dispatch.constantHeading(0.3, 0, -35, 0.001, 0, 0.0003);
                dispatch.constantHeading(0.3, 0, 1, 0.001, 0, 0.0003);
                dispatch.turnAbsPID(0, 1);
                dispatch.constantHeading(0.5, 4, 0, 0.001, 0, 0.0003);
                dispatch.constantHeading(0.45, 10, -30, 0.001, 0, 0.0003);

                dispatch.spinCarousel(1400);
                dispatch.constantHeading(0.2, 0, -6, 2.5, 0.001, 0, 0.0003);
                dispatch.spinCarousel(0);
                dispatch.constantHeading(0.5, 5, 0, 0.001, 0, 0.0003);
                dispatch.constantHeading(0.3, 0, 24, 0.001, 0, 0.0003);
                dispatch.turnAbsPID(-90, 1);
                */

                break;
            }
        }


        phoneCam.stopStreaming();
        telemetry.update();
    }
}