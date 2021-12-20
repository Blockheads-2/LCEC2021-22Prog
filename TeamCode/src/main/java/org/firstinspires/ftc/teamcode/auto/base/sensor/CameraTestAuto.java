package org.firstinspires.ftc.teamcode.auto.base.sensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.auto.cv.CoreDetection;

@Disabled
@Autonomous(name="Camera Test Auto", group="Robot Base Drive")
public class CameraTestAuto extends LinearOpMode {
    OpenCvCamera phoneCam;
    HardwareDrive         robot   = new HardwareDrive();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    Constants constants = new Constants();

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Robot Motor
        robot.init(hardwareMap);

        //Initialize CV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        CoreDetection detector = new CoreDetection(telemetry);
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
            case LEFT:
                //...
                break;
            case RIGHT:
                //...
                break;
            case MID:
                //...
                break;
            case NOT_FOUND:
                //...
                break;
        }

        phoneCam.stopStreaming();
    }
}