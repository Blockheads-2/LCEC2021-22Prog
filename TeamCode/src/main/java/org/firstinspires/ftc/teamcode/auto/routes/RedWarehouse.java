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
                dispatch.moveElevator(constants.elevatorPositionBottom);
                sleep(750);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.5,-5,16,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 1000);
                dispatch.constantHeading(0.5,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(0.3,0,-18,0.001,0,0.0003);
                dispatch.turnAbsPID(90, 1);
                dispatch.constantHeading(0.2,5,0,0.001,0,0.0003);

                //Cycle 1
                dispatch.spinIntake(1);
                dispatch.constantHeading(0.75,0,33,0.001,0,0.0003);
                dispatch.variableHeading(.5, 4, 0, .5);
                dispatch.variableHeading(.5, -4, 0, .5);
                dispatch.spinIntake(-0.3);
                dispatch.constantHeading(.5, 0, -5, 0.001,0,0.0003);
                dispatch.spinIntake(.15);
                dispatch.turnAbsPID(-90,1);
                dispatch.constantHeading(1,-20,0,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionTop);
                dispatch.constantHeading(0.75,-9,14,0.001,0,0.0003);
                dispatch.variableHeading(0.75,30,14,2.5);
                dispatch.spinIntake(0);
                dispatch.spinIntake(-1,1250);
                dispatch.variableHeading(0.75,18 ,-12,2);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.75,0,-17,0.001,0,0.0003);
                dispatch.turnAbsPID(-270,2);
                dispatch.constantHeading(.5,5,0,0.001,0,0.0003);


                //Cycle 2
                dispatch.spinIntake(1);
                dispatch.constantHeading(0.75,0,10,0.001,0,0.0003);
                dispatch.variableHeading(.5, -4, 0, .5);
                dispatch.variableHeading(.5, 4, 0, .5);
                dispatch.spinIntake(-0.3);
                dispatch.constantHeading(.5, 0, -7, 0.001,0,0.0003);
                dispatch.spinIntake(.15);
                dispatch.turnAbsPID(-90,1);
                dispatch.spinIntake(.15);
                dispatch.constantHeading(1,-20,0,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionTop);
                dispatch.constantHeading(0.75,-9,19,0.001,0,0.0003);
                dispatch.variableHeading(0.75,30,14,2);
                dispatch.spinIntake(0);
                dispatch.spinIntake(-1,1250);
                dispatch.variableHeading(0.75, 18 ,-12,2);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.75,0,-26,0.001,0,0.0003);

                dispatch.turnAbsPID(90,1);
                dispatch.constantHeading(0.75,0,-0,0.001,0,0.0003);

                break;
            }
            case RIGHT: {
                dispatch.moveElevator(constants.elevatorPositionTop);
                sleep(750);
                // move to drop
                dispatch.spinIntake(0.1);
                dispatch.variableHeading(0.5,-6.5,15,2);
                dispatch.spinIntake(0);

                //out-take
                dispatch.spinIntake(-1, 1000);
                dispatch.constantHeading(0.5,0,-5,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.turnAbsPID(0,0.5);
                dispatch.constantHeading(0.5,0,-18,0.001,0,0.0003);
                dispatch.turnAbsPID(90, 1);
                dispatch.constantHeading(0.5,5,0,0.001,0,0.0003);
                dispatch.constantHeading(0.5, 0, 17, 0.001,0,0.0003);

                //Cycle 1
                dispatch.spinIntake(1);
                dispatch.constantHeading(0.75,0,23,0.001,0,0.0003);
                dispatch.variableHeading(.5, -4, 0, .5);
                dispatch.variableHeading(.5, 4, 0, .5);
                dispatch.constantHeading(.5, 0, -5, 0.001,0,0.0003);
                dispatch.spinIntake(0.15);
                dispatch.turnAbsPID(-90,1);
                dispatch.constantHeading(1,-20,0,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionTop);
                dispatch.constantHeading(0.75,-9,16,0.001,0,0.0003);
                dispatch.variableHeading(0.75,30,14,2.5);
                dispatch.spinIntake(0);
                dispatch.spinIntake(-1,1250);
                dispatch.variableHeading(0.75,18 ,-12,2);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.75,0,-17,0.001,0,0.0003);
                dispatch.turnAbsPID(90,2);
                dispatch.constantHeading(.5,5,0,0.001,0,0.0003);


                //Cycle 2
                dispatch.spinIntake(1);
                dispatch.constantHeading(0.75,0,10,0.001,0,0.0003);
                dispatch.variableHeading(.5, -4, 0, .5);
                dispatch.variableHeading(.5, 4, 0, .5);
                dispatch.constantHeading(.5, 0, -7, 0.001,0,0.0003);
                dispatch.spinIntake(0.15);
                dispatch.turnAbsPID(-90,1);
                dispatch.constantHeading(1,-20,0,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionTop);
                dispatch.constantHeading(0.75,-9,19,0.001,0,0.0003);
                dispatch.variableHeading(0.75,30,16,3);
                dispatch.spinIntake(-1,1250);
                dispatch.variableHeading(0.75, 18 ,-12,2);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.75,0,-23,0.001,0,0.0003);
                dispatch.turnAbsPID(90,1);
                dispatch.constantHeading(0.75,0,-0,0.001,0,0.0003);

                break;
            }
            case MID: {
                //power on lift
                dispatch.moveElevator(constants.elevatorPositionTop - 200);
                sleep(500);
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
                dispatch.constantHeading(0.5, 0, 17, 0.001,0,0.0003);

                //Cycle 1
                dispatch.spinIntake(1);
                dispatch.constantHeading(0.75,0,23,0.001,0,0.0003);
                dispatch.variableHeading(.5, -4, 0, .5);
                dispatch.variableHeading(.5, 4, 0, .5);
                dispatch.constantHeading(.5, 0, -5, 0.001,0,0.0003);
                dispatch.spinIntake(0.15);
                dispatch.turnAbsPID(-90,1);
                dispatch.constantHeading(1,-20,0,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionTop);
                dispatch.constantHeading(0.75,-9,16,0.001,0,0.0003);
                dispatch.variableHeading(0.75,30,12,2.5);
                dispatch.spinIntake(0);
                dispatch.spinIntake(-1,1250);
                dispatch.variableHeading(0.75,18 ,-13,2);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.75,0,-17,0.001,0,0.0003);
                dispatch.turnAbsPID(90,2);
                dispatch.constantHeading(.5,5,0,0.001,0,0.0003);

                //Cycle 2
                dispatch.spinIntake(1);
                dispatch.constantHeading(0.75,0,10,0.001,0,0.0003);
                dispatch.variableHeading(.5, -4, 0, .5);
                dispatch.variableHeading(.5, 4, 0, .5);
                dispatch.constantHeading(.5, 0, -7, 0.001,0,0.0003);
                dispatch.turnAbsPID(-90,1);
                dispatch.constantHeading(1,-20,0,0.001,0,0.0003);
                dispatch.moveElevator(constants.elevatorPositionTop);
                dispatch.constantHeading(0.75,-9,19,0.001,0,0.0003);
                dispatch.variableHeading(0.75,30,14,3);
                dispatch.spinIntake(-1,1250);
                dispatch.variableHeading(0.75, 16,-12,2);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.75,0,-23,0.001,0,0.0003);
                dispatch.turnAbsPID(90,1);
                dispatch.constantHeading(0.75,0,-0,0.001,0,0.0003);

                break;
            }
        }
        phoneCam.stopStreaming();
        telemetry.update();
    }
}

