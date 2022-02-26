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

@Autonomous(name="Red Warehouse", group="Routes")
public class RedWarehouse extends LinearOpMode {

    /* Declare OpMode members. */
    OpenCvCamera phoneCam;
    AutoHub dispatch;
    Constants constants = new Constants();

    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

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
                dispatch.moveElevator(constants.elevatorPositionBottom - 250);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.5,-6.75,15,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 750);
                dispatch.constantHeading(0.5,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(0.7,0,-18,0.001,0,0.0003);
                dispatch.turnAbsPID(90, 1);
                dispatch.constantHeading(0.7,8,0,0.001,0,0.0003);
                //delete later
                //dispatch.constantHeading(0.75,0,23,0.001,0,0.0003);
                break;
            }
            case RIGHT: {
                dispatch.moveElevator(constants.elevatorPositionTop);
                sleep(1000);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.55,-6.5,15.5, 2);
                dispatch.spinIntake(0);

                //out-take
                //dispatch.constantHeading(0.75,0,0.75,0.25,0.001,0,0.0003);
                dispatch.constantHeading(0.75,0,0.75,0.001,0,0.0003);
                dispatch.spinIntake(-1, 700);
                dispatch.constantHeading(0.75,0,0.75,0.001,0,0.0003);
                dispatch.constantHeading(1,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(1,5,0,0.001,0,0.0003);
                dispatch.constantHeading(1,0,-18,0.001,0,0.0003);
                dispatch.turnAbsPID(90, 1);
                dispatch.constantHeading(1,8,0,0.001,0,0.0003);
                //delete later
                //dispatch.constantHeading(0.75,0,23,0.001,0,0.0003);


                break;
            }
            case MID: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop - 200);
                sleep(1000);
                // move to drop
                dispatch.variableHeading(0.5,-5,12,2);

                //out-take
                dispatch.spinIntake(-1, 1000);
                dispatch.constantHeading(0.5,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(0.3,0,-18,0.001,0,0.0003);
                dispatch.turnAbsPID(90, 1);
                dispatch.constantHeading(0.2,5,0,0.001,0,0.0003);
                //dispatch.constantHeading(0.5, 0, 17, 0.001,0,0.0003);
                //delete later
                //dispatch.constantHeading(0.75,0,23,0.001,0,0.0003);
            }
        }
        //Cycle 1
        dispatch.constantHeading(.75,5,23,0.001,0,0.0003);
        dispatch.spinIntake(1);
        dispatch.constantHeading(1,0,22,0.001,0,0.0003);
        dispatch.constantHeading(.5,0,5,0.001,0,0.0003);
        if (!AutoHub.finishedIntake)
            dispatch.turn(-10);
        if (!AutoHub.finishedIntake)
            dispatch.turn(10);
        dispatch.constantHeading(1, 0, -12, .001, 0, .0003);
        AutoHub.finishedIntake = false;
        dispatch.spinIntake(.15);
        AutoHub.finishedIntake = false;
        AutoHub.over = false;
        AutoHub.checkOver = false;
        AutoHub.checkOver2 = false;
        dispatch.turnAbsPID(270,1);
        dispatch.constantHeading(.75,-20,0,0.001,0,0.0003);
        dispatch.moveElevator(constants.elevatorPositionTop);
        dispatch.constantHeading(0.75,-5,25,true,0.001,0,0.0003);
        dispatch.variableHeading(0.75,26,10,2 );
        dispatch.spinIntake(-1,700);
        dispatch.variableHeading(0.75,24 ,-9,2); // 18 -5
        dispatch.moveElevator(constants.elevatorPositionDown);
        dispatch.turnAbsPID(-90,0.1);
        dispatch.constantHeading(.5, -6, 0, 0.001,0,0.0003);
        dispatch.constantHeading(.75,-20,0,true,0.001,0,0.0003);
        dispatch.constantHeading(1,-3,-23,0.001,0,0.0003);
        dispatch.turnAbsPID(90,1);


        //Cycle 2
        dispatch.spinIntake(1);
        dispatch.constantHeading(.75, 0, 10, .001, 0, .0003);
        if (!AutoHub.finishedIntake)
            dispatch.turn(-10);
        if (!AutoHub.finishedIntake)
            dispatch.turn(10);
        dispatch.constantHeading(1,0,-10,0.001,0,0.003);
        AutoHub.finishedIntake = false;
        AutoHub.over = false;
        AutoHub.checkOver = false;
        AutoHub.checkOver2 = false;
        dispatch.spinIntake(.2);
        dispatch.turnAbsPID(-90,1);
        dispatch.constantHeading(1,-25,0,0.001,0,0.0003);
        dispatch.moveElevator(constants.elevatorPositionTop);
        dispatch.constantHeading(0.75,-9,30,true,0.001,0,0.0003);
        dispatch.variableHeading(0.75,26,10,2);
        dispatch.spinIntake(-1,700);
        dispatch.variableHeading(0.75,24 ,-9,2); //18 -5
        dispatch.moveElevator(constants.elevatorPositionDown);
        dispatch.turnAbsPID(-90,0.1);
        dispatch.constantHeading(1,-15,0,0.001,0,0.0003);
        dispatch.constantHeading(1,-2,-28,2, 0.001,0,0.0003);
        phoneCam.stopStreaming();
        telemetry.update();
    }
}

