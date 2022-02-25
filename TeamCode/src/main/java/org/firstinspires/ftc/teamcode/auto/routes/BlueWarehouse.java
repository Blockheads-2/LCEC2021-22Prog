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

@Autonomous(name = "Blue Warehouse", group = "Routes")
public class BlueWarehouse extends LinearOpMode {

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
                dispatch.moveElevator(constants.elevatorPositionBottom);
                sleep(750);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.65,8.5,13,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 550);
                dispatch.constantHeading(0.75,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(0.75,0,-20,0.001,0,0.0003);
                dispatch.turnAbsPID(-90,1.2);
                dispatch.constantHeading(1,-10,0,0.001,0,0.0003);
                //dispatch.constantHeading(0.8, 0, 17,true, 0.001,0,0.0003);
                //delete later
                //dispatch.constantHeading(1,0,23,0.001,0,0.0003);

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
                //dispatch.constantHeading(1, 0, 23, 0.001,0,0.0003);
                //delete this line later
                //dispatch.constantHeading(1,0,23,0.001,0,0.0003);


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
        dispatch.constantHeading(1,-7,42,0.00,0,0.000);
        AutoHub.over = false;
        AutoHub.checkOver = false;
        AutoHub.checkOver2 = false;
        dispatch.spinIntake(1);
        sleep(350);
        if (!AutoHub.finishedIntake)
            dispatch.variableHeading(.7, 4, 0, .4);
        if (!AutoHub.finishedIntake)
            dispatch.variableHeading(.7, -4, 0, .4);
        dispatch.constantHeading(.7, 0, -9, 0.001,0,0.0003);
        AutoHub.finishedIntake = false;
        dispatch.turnAbsPID(87,1);
        dispatch.constantHeading(.5,25,0,0.001,0,0.0003);
        AutoHub.over = false;
        AutoHub.checkOver = false;
        AutoHub.checkOver2 = false;
        dispatch.moveElevator(constants.elevatorPositionTop);
        dispatch.constantHeading(0.5,9,30,true,0.001,0,0.0003);
        AutoHub.over = false;
        AutoHub.checkOver = false;
        AutoHub.checkOver2 = false;
        dispatch.variableHeading(0.75,-26,10,2); //24.5 14
        dispatch.spinIntake(-1,750);
        dispatch.variableHeading(0.75,-25 ,-6.5,2);
        dispatch.moveElevator(constants.elevatorPositionDown);
        dispatch.turnAbsPID(90,.2);
        dispatch.constantHeading(1,7,0,0.001,0,0.0003);
        dispatch.constantHeading(1,0,-17,0.001,0,0.0003);
        dispatch.turnAbsPID(-90,1);
        dispatch.constantHeading(.5,0,8,0.001,0,0.0003);



        //Cycle 2
        dispatch.spinIntake(1);
        if (!AutoHub.finishedIntake)
            dispatch.variableHeading(.7, 4, 0, .4);
        if (!AutoHub.finishedIntake)
            dispatch.variableHeading(.7, -4, 0, .4);
        dispatch.constantHeading(.7, 0, -9, 0.001,0,0.0003);
        dispatch.turnAbsPID(90,1);
        dispatch.constantHeading(1,25,0,0.001,0,0.0003);
        dispatch.moveElevator(constants.elevatorPositionTop);
        AutoHub.over = false;
        AutoHub.checkOver = false;
        AutoHub.checkOver2 = false;
        dispatch.constantHeading(0.5,9,32,true,0.001,0,0.0003);
        dispatch.variableHeading(.75,-26,10,2);
        dispatch.spinIntake(-1,750);
        dispatch.variableHeading(.75,-24 ,-6.5,2);
        dispatch.moveElevator(constants.elevatorPositionDown);
        dispatch.turnAbsPID(90,0.2);
        dispatch.constantHeading(1,8,0,0.001,0,0.0003);
        dispatch.turnAbsPID(90,0.2);
        dispatch.constantHeading(1,0,-40,true,0.001,0,0.0003);
        dispatch.constantHeading(0.5,0,-16,2,0.001,0,0.0003);


        phoneCam.stopStreaming();
        telemetry.update();
    }
}
