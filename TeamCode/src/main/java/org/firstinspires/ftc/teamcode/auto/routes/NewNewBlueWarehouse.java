package org.firstinspires.ftc.teamcode.auto.routes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.cv.BlueWarehouseDetection;
import org.firstinspires.ftc.teamcode.auto.cv.CoreDetection;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.common.Constants;

@Autonomous(name = "New New Blue Warehouse", group = "New")
public class NewNewBlueWarehouse extends LinearOpMode {

    OpenCvCamera phoneCam;
    AutoHub dispatch;
    Constants constants = new Constants();
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

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
        long startTime = System.currentTimeMillis();
        long elapsedTime = System.currentTimeMillis() - startTime;
        long elapsedSeconds = elapsedTime / 1000;
        switch (detector.getLocation()) {
            case LEFT: {
                dispatch.moveElevator(constants.elevatorPositionBottom - 200);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.65,8.3,14.7,2);

                //out-take
                dispatch.spinIntake(-1, 750);

                //dispatch.variableHeading(1,-25 ,-33,2);
                dispatch.turnAbsPID(-90,0.6);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(1,-45,0,0.001,0,0.0003);
                dispatch.spinIntake(1);
                dispatch.constantHeading(1,-4,32,0.001,0,0.0003);
                dispatch.constantHeading(0.75,0,8,0.001,0,0.0003);


                break;

            }
            case RIGHT: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop);
                sleep(1000);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.5,7.5,16.5,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 2000);
                dispatch.constantHeading(0.75,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(0.75,0,-18,0.001,0,0.0003);
                dispatch.turnAbsPID(-90, 1);
                dispatch.constantHeading(0.75,-5,0,0.001,0,0.0003);



                break;
            }
            case MID: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop - 200);
                sleep(800);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.75,6,12,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 1000);
                dispatch.constantHeading(0.75,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(0.75,0,-18,0.001,0,0.0003);
                dispatch.turnAbsPID(-90, 1);
                dispatch.constantHeading(0.75,-8,0,0.001,0,0.0003);
                //dispatch.constantHeading(1, 0, 17, 0.001,0,0.0003);
                //delete line later
                //dispatch.constantHeading(1,0,23,0.001,0,0.0003);

                break;
            }

        }
        //Cycle 1
        int i = 0;
        while (i < 3 && (elapsedSeconds < 20)){
            dispatch.spinIntake(1);
            if (!AutoHub.finishedIntake)
                dispatch.turnPID(10, 0.5);
            if (!AutoHub.finishedIntake)
                dispatch.turnPID(-10, 0.5);
            dispatch.constantHeading(1, 0, -9, 0.001, 0, 0.0003);
            AutoHub.finishedIntake = false;

            dispatch.constantHeading(1,-50,0,0.3,0,0,0);
            dispatch.constantHeading(1, -5,-26-(i*1.5),true, 0.001, 0, 0.0003);
            dispatch.moveElevator(constants.elevatorPositionTop);
            dispatch.spinIntake(0.2);
            dispatch.constantHeading(1, -2, -6.5, 0.001, 0, 0.0003);
            dispatch.turnAbsPID(30,1);
            dispatch.spinIntake(-0.05);
            dispatch.constantHeading(1, 0, 19, 0.001, 0, 0.0003);
         //   dispatch.constantHeading(1, 25, 0, 0.001, 0, 0.0003);
            //dispatch.turnAbsPID(14, 1);
            dispatch.spinIntake(-1, 750);
            //dispatch.turnAbsPID(-90, 1);
            dispatch.constantHeading(1, 0, -5, 0.001, 0, 0.0003);
            dispatch.turnAbsPID(-90,1);
            dispatch.moveElevator(constants.elevatorPositionDown);
            dispatch.constantHeading(1, -40, 40, 0.001, 0, 0.0003);
            dispatch.constantHeading(1,0,24,true,0.001, 0, 0.0003);
            dispatch.constantHeading(1,0,13+(i*1.5),0.001, 0, 0.0003);
            elapsedTime = System.currentTimeMillis() - startTime;
            elapsedSeconds = elapsedTime / 1000;
            i++;
        }
        /*
        AutoHub.over = false;
        AutoHub.checkOver = false;
        AutoHub.checkOver2 = false;
        if (!AutoHub.finishedIntake)
            dispatch.turn(10);
        if (!AutoHub.finishedIntake)
            dispatch.turn(-10);
        dispatch.constantHeading(1, 0, -9, 0.001,0,0.0003);
        AutoHub.finishedIntake = false;


        dispatch.constantHeading(1,-20,0,0.001,0,0.0003);
        AutoHub.over = false;
        AutoHub.checkOver = false;
        AutoHub.checkOver2 = false;
        dispatch.moveElevator(constants.elevatorPositionTop);
        dispatch.spinIntake(0.2);
        dispatch.constantHeading(1,-5,-30,true,0.001,0,0.0003);
        dispatch.constantHeading(1,0,-10, 0.001, 0, 0.0003);
        dispatch.constantHeading(1, 28,-10, 0.001, 0, 0.0003);
        dispatch.turnAbsPID(50, 0.7);
        dispatch.spinIntake(0);
        dispatch.spinIntake(-1,2000);
        phoneCam.stopStreaming();
        telemetry.update();
        */
    }
}
