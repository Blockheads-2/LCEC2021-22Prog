package org.firstinspires.ftc.teamcode.auto.dispatch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.cv.CoreDetection;
import org.firstinspires.ftc.teamcode.auto.cv.Duck_Detection;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Button;


@Autonomous(name = "Camera Merge", group = "Test")
@Disabled
public class TestCameraRoute extends LinearOpMode{

    OpenCvCamera phoneCam;
    AutoHub dispatch;
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();
    Duck_Detection detector = new Duck_Detection(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
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



        phoneCam.stopStreaming();
    }
}
