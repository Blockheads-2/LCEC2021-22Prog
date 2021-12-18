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
                dispatch.moveElevator(constants.elevatorPositionBottom - 400);

                // move to drop
                dispatch.spinIntake(0.2);
                dispatch.constantHeading(0.5,25,30,2.5,0.001,0,0.0003);
                dispatch.variableHeading(0.6,-10,10,1.5);
                dispatch.turnPID(-10,1);
                dispatch.constantHeading(0.3,0,3,1,0.001,0,0.0003);

                //out-take
                dispatch.spinIntake(-0.75,2000);
                dispatch.constantHeading(0.5,0,-1,0.4,0.001,0,0.0003);


                dispatch.moveElevator(constants.elevatorPositionDown);

                //move to carousel
                dispatch.constantHeading(0.5,4,0,0.7,0.001,0,0.0003);
                dispatch.turnAbsPID(270,1.5);
                dispatch.constantHeading(0.5,0,-36,2,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,1,1,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.45,5,-34,3.7,0.001,0,0.0003);

                dispatch.spinCarousel(1600);
                dispatch.constantHeading(0.1,0,-2,4,0.001,0,0.0003);
                dispatch.spinCarousel(0);

                dispatch.constantHeading(0.5,3,20,4,0.001,0,0.0003);

                telemetry.addLine("Path: Left");
                break;
            }
            case RIGHT: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop);

                // move to drop
                dispatch.variableHeading(0.5,-6.25,17.1,1.5);
                dispatch.constantHeading(0.5,0,1.5,0.4,0.001,0,0.0003);

                //outtake
                robot.spin.setPower(-1);
                sleep(2300);

                //park
                dispatch.constantHeading(0.5,0,-1.5,0.4,0.001,0,0.0003);
                dispatch.variableHeading(0.5,-6.25,-17.1,1.5);
                dispatch.moveElevator(constants.elevatorPositionDown);

                //carousel
                dispatch.variableHeading(0.5,20,-20,2);
                dispatch.constantHeading(0.5,0,-48,2.5,0.001,0,0.0003);

                dispatch.constantHeading(0.3,-7,0,1.5,0.001,0,0.0003);
                dispatch.spinCarousel(1600);
                dispatch.constantHeading(0.1,-2,0,4,0.001,0,0.0003);
                dispatch.spinCarousel(0);

                dispatch.constantHeading(0.5,22,0,2,0.001,0,0.0003);
                dispatch.constantHeading(0.5,0,-5,2,0.001,0,0.0003);

                break;
            }
            case MID: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionMid);

                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.constantHeading(0.6,30,20,2,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,0,20,1.5,0.001,0,0.0003);
                dispatch.variableHeading(0.6,-10,2,1.5);
                dispatch.constantHeading(0.5,4,1,0.7,0.001,0,0.0003);

                //out-take
                dispatch.spinIntake(-1,2300);
                dispatch.constantHeading(0.5,0,-5,0.7,0.001,0,0.0003);


                dispatch.moveElevator(constants.elevatorPositionDown);

                //move to carousel -- start ---
                dispatch.turnAbsPID(270,1.5);
                dispatch.constantHeading(0.3,0,-35,3,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,1,1,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,4,0,0.5,0.001,0,0.0003);
                dispatch.constantHeading(0.45,10,-40,3.7,0.001,0,0.0003);

                dispatch.spinCarousel(1600);
                dispatch.constantHeading(0.1,0,-2,4,0.001,0,0.0003);
                dispatch.spinCarousel(0);

                dispatch.constantHeading(0.5,3,20,4,0.001,0,0.0003);

                break;
            }

        }

        phoneCam.stopStreaming();
        telemetry.update();
    }
}
