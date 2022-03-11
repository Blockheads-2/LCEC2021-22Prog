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

@Autonomous(name = "New Red Warehouse", group = "New")
public class NewRedWarehouse extends LinearOpMode {

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
        switch (detector.getLocation()) {
            case LEFT: {
                dispatch.moveElevator(constants.elevatorPositionBottom - 650);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.65,-8.25,16.5,2);

                //out-take
                dispatch.spinIntake(-1, 1000);
                dispatch.spinIntake(1, 500);
                dispatch.spinIntake(-1, 1000);

                //dispatch.variableHeading(1,-25 ,-33,2);
                dispatch.turnAbsPID(90,0.6);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(1,45,0,0.001,0,0.0003);
                dispatch.spinIntake(1);
                dispatch.constantHeading(.75,5,32,0.001,0,0.0003);
                dispatch.constantHeading(0.75,0,10,0.001,0,0.0003);


                break;

            }
            case RIGHT: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop);
                sleep(1000);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.5,-7.5,16.5,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 1000);
                dispatch.constantHeading(0.75,0,-5,0.001,0,0.0003);
                dispatch.turnAbsPID(-90,0.6);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(1,45,0,0.001,0,0.0003);
                dispatch.spinIntake(1);
                dispatch.constantHeading(1,4,32,0.001,0,0.0003);
                dispatch.constantHeading(0.75,0,10,0.001,0,0.0003);



                break;
            }
            case MID: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop - 200);
                sleep(800);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.75,-6,12,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 1000);
                dispatch.constantHeading(0.75,0,-5,0.001,0,0.0003);
                dispatch.turnAbsPID(90,0.6);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(1,45,0,0.001,0,0.0003);
                dispatch.spinIntake(1);
                dispatch.constantHeading(1,4,32,0.001,0,0.0003);
                dispatch.constantHeading(0.75,0,10,0.001,0,0.0003);
                //dispatch.constantHeading(1, 0, 17, 0.001,0,0.0003);
                //delete line later
                //dispatch.constantHeading(1,0,23,0.001,0,0.0003);

                break;
            }

        }
        //Cycle 1
        for (int i = 0; i < 2; i++) {
            dispatch.spinIntake(1);
            if (!AutoHub.finishedIntake)
                dispatch.turnPID(-15, 0.5);
            if (!AutoHub.finishedIntake)
                dispatch.turnPID(15, 0.5);
            dispatch.constantHeading(1, 0, -9, 0.001, 0, 0.0003);
            AutoHub.finishedIntake = false;

            dispatch.constantHeading(1, 20, 0, 0.001, 0, 0.0003);
            dispatch.moveElevator(constants.elevatorPositionTop);
            dispatch.spinIntake(0.2);
            dispatch.constantHeading(1, 5, -22-(i*3), true, 0.001, 0, 0.0003);
            dispatch.constantHeading(1, 0, -20, 0.001, 0, 0.0003);
            dispatch.turnAbsPID(0, 1);
            dispatch.spinIntake(-0.15);
            dispatch.constantHeading(1, 0, 16, 0.001, 0, 0.0003);
            dispatch.spinIntake(-1, 1000);
            dispatch.spinIntake(1, 500);
            dispatch.spinIntake(-1, 1000);
            dispatch.constantHeading(1, 0, -15, 0.001, 0, 0.0003);
            dispatch.turnAbsPID(90, 1);
            dispatch.moveElevator(constants.elevatorPositionDown);
            dispatch.constantHeading(.75, 15, 0, 0.001, 0, 0.0003);
            /*dispatch.constantHeading(1, -4, 48+(i*4), true, 0.001, 0, 0.0003);
            if(AutoHub.over){
                dispatch.constantHeading(1, 0, 22+(i*4), 0.001, 0, 0.0003);
            }
            */
            dispatch.constantHeading(1, 8, 50+(3*i), 0.001, 0, 0.0003);
            dispatch.constantHeading(.75, 3, 10, 0.001, 0, 0.0003);
            dispatch.spinIntake(1);
            dispatch.constantHeading(.3, 0, 5, 0.001, 0, 0.0003);
        }
        dispatch.spinIntake(1);
        dispatch.turnPID(-15, 0.5);
        dispatch.turnPID(15, 0.5);
        dispatch.constantHeading(1, 0, -6, 0.001, 0, 0.0003);

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
