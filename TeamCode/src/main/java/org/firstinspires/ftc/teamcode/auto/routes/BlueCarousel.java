package org.firstinspires.ftc.teamcode.auto.routes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.cv.BlueDetection;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.common.Constants;

@Autonomous(name = "Blue Carousel", group = "Routes")
//@Disabled
public class BlueCarousel extends LinearOpMode{

    OpenCvCamera phoneCam;
    AutoHub dispatch;
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();

    @Override
    public void runOpMode() throws InterruptedException {
    dispatch = new AutoHub(this);

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());
    phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    BlueDetection detector = new BlueDetection(telemetry);
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
                dispatch.moveElevator(constants.elevatorPositionBottom - 150);

                // move to drop
                dispatch.spinIntake(0.2);
                dispatch.constantHeading(0.6,20,20,2,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);

                dispatch.constantHeading(0.5,0,16,1.5,0.001,0,0.0003);
                dispatch.spinIntake(0);
                dispatch.variableHeading(0.6,-10,2,1.5);
                dispatch.constantHeading(0.5,0,1.75,0.7,0.001,0,0.0003);
                dispatch.spinIntake(-0.7,1200);
                dispatch.constantHeading(0.5,0,-5,0.7,0.001,0,0.0003);


                dispatch.moveElevator(constants.elevatorPositionDown);

                //move to carousel -- start ---
                dispatch.turnAbsPID(270,1.5);
                dispatch.constantHeading(0.3,0,-35,1.7,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,1,0.5,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,4,0,0.5,0.001,0,0.0003);
                dispatch.constantHeading(0.45,10,-25,2.5,0.001,0,0.0003);

                dispatch.spinCarousel(1400);
                dispatch.constantHeading(0.2,0,-6,2.7,0.001,0,0.0003);
                dispatch.spinCarousel(0);
                dispatch.constantHeading(0.5,5,0,0.35,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,26,1.7,0.001,0,0.0003);
                dispatch.turnAbsPID(-90,1);

                break;
            }
            case RIGHT: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop);
                sleep(1500);

                // move to drop
                dispatch.variableHeading(0.5,-6.25,15,2);
                dispatch.constantHeading(0.5,0,1.5,1,0.001,0,0.0003);

                //outtake
                dispatch.spinIntake(-1, 2000);

                //park
                dispatch.variableHeading(0.5,20,-12, 1.5);
                dispatch.constantHeading(0.25, 0, -5, .5, .001, 0, .0003);

                dispatch.moveElevator(constants.elevatorPositionDown);

                dispatch.turnAbsPID(0,1);

                dispatch.constantHeading(.5,0, 5, .5, .001, 0, .0003);
                dispatch.constantHeading(0.5,12,0,2,0.001,0,0.0003);

                dispatch.spinCarousel(1400);
                dispatch.constantHeading(0.2,0,-6,3,0.001,0,0.0003);
                dispatch.spinCarousel(0);
                dispatch.constantHeading(0.5,5,0,1,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,26,2,0.001,0,0.0003);
                dispatch.turnAbsPID(-90,1);

                //carousel


                break;
            }
            case MID: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionMid - 350);

                // move to drop
                dispatch.spinIntake(0.2);
                dispatch.constantHeading(0.6,30,20,2,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,0,16,1.5,0.001,0,0.0003);
                dispatch.variableHeading(0.6,-10,2,1.5);
                dispatch.constantHeading(0.5,-5.5,0,0.7,0.001,0,0.0003);
                dispatch.spinIntake(0);
                dispatch.constantHeading(0.5,0,.8,0.7,0.001,0,0.0003);


                //out-take
                dispatch.turnAbsPID(-90,1);
                dispatch.constantHeading(0.5,0,6.5,0.6,0.001,0,0.0003);
                dispatch.spinIntake(-1,2300);
                dispatch.constantHeading(0.5,0,-5,0.7,0.001,0,0.0003);


                dispatch.moveElevator(constants.elevatorPositionDown);

                //move to carousel -- start ---
                dispatch.turnAbsPID(270,1.5);
                dispatch.constantHeading(0.3,0,-35,3,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,1,1,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,4,0,0.5,0.001,0,0.0003);
                dispatch.constantHeading(0.45,10,-25,3.7,0.001,0,0.0003);

                dispatch.spinCarousel(1400);
                dispatch.constantHeading(0.2,0,-6,3,0.001,0,0.0003);
                dispatch.spinCarousel(0);
                dispatch.constantHeading(0.5,5,0,1,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,26,2,0.001,0,0.0003);
                dispatch.turnAbsPID(-90,2);



                break;
            }
        }

        phoneCam.stopStreaming();
        telemetry.update();
    }
}
