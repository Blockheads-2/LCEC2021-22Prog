package org.firstinspires.ftc.teamcode.auto.routes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auto.cv.CoreDetection;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.positioning.MathConstHead;
import org.firstinspires.ftc.teamcode.common.positioning.MathSpline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Red Warehouse", group="Routes")

//@Disabled
public class RedWarehouse extends LinearOpMode {

    /* Declare OpMode members. */
    OpenCvCamera phoneCam;
    HardwareDrive robot   = new HardwareDrive();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    Constants constants = new Constants();
    MathSpline mathSpline = new MathSpline();
    MathConstHead mathConstHead = new MathConstHead();

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


        waitForStart();


        switch (detector.getLocation()) {
            case LEFT: {


                //power on lift
                dispatch.moveElevator(constants.elevatorPositionMid - 430);

                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.6,-5,12,1.5);

                dispatch.spinIntake(-1,2000);

                dispatch.variableHeading(0.5,-20,-28,2);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(90,1);
                dispatch.constantHeading(0.5,8,0,0,0,0);

                dispatch.constantHeading(0.5,0,20,0.001,0,0.0003);

                break;
            }
            case RIGHT: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop);

                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.constantHeading(0.6,30,20,2,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,0,21,1.5,0.001,0,0.0003);
                dispatch.variableHeading(0.6,-10,1.7,1.5);
                dispatch.constantHeading(0.5,-5.5,0,0.7,0.001,0,0.0003);
                dispatch.spinIntake(0);
                dispatch.constantHeading(0.5,0,.8,0.7,0.001,0,0.0003);


                //out-take
                dispatch.constantHeading(0.5,2,10,0.7,0.001,0,0.0003);
                dispatch.spinIntake(-1,2000);

                dispatch.constantHeading(0.5,0,-5,0.7,0.001,0,0.0003);

                dispatch.moveElevator(constants.elevatorPositionDown);
                //move to carousel -- start ---
                dispatch.turnAbsPID(270,0.5);
                dispatch.constantHeading(0.3,0,-35,2,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,1,1,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,4,0,0.5,0.001,0,0.0003);
                dispatch.constantHeading(0.45,10,-30,3.5,0.001,0,0.0003);

                dispatch.spinCarousel(1400);
                dispatch.constantHeading(0.2,0,-6,2.5,0.001,0,0.0003);
                dispatch.spinCarousel(0);
                dispatch.constantHeading(0.5,5,0,1,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,26,2,0.001,0,0.0003);
                dispatch.turnAbsPID(-90,1);

                break;
            }
            case MID: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop);

                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.constantHeading(0.6,30,20,2,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,0,21,1.5,0.001,0,0.0003);
                dispatch.variableHeading(0.6,-10,1.7,1.5);
                dispatch.constantHeading(0.5,-5.5,0,0.7,0.001,0,0.0003);
                dispatch.spinIntake(0);
                dispatch.constantHeading(0.5,0,.8,0.7,0.001,0,0.0003);


                //out-take
                dispatch.constantHeading(0.5,2,5.8,0.7,0.001,0,0.0003);

                dispatch.spinIntake(-1,2000);

                dispatch.constantHeading(0.5,0,-5,0.7,0.001,0,0.0003);

                dispatch.moveElevator(constants.elevatorPositionDown);
                //move to carousel -- start ---
                dispatch.turnAbsPID(270,0.5);
                dispatch.constantHeading(0.3,0,-35,2,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,1,1,0.001,0,0.0003);
                dispatch.turnAbsPID(0,1);
                dispatch.constantHeading(0.5,4,0,0.5,0.001,0,0.0003);
                dispatch.constantHeading(0.45,10,-30,3.5,0.001,0,0.0003);

                dispatch.spinCarousel(1400);
                dispatch.constantHeading(0.2,0,-6,2.5,0.001,0,0.0003);
                dispatch.spinCarousel(0);
                dispatch.constantHeading(0.5,5,0,1,0.001,0,0.0003);
                dispatch.constantHeading(0.3,0,26,2,0.001,0,0.0003);
                dispatch.turnAbsPID(-90,1);



                break;
            }
        }
        phoneCam.stopStreaming();
        telemetry.update();
    }
}

