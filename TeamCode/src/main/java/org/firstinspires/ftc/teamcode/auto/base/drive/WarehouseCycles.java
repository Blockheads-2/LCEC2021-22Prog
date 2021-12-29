package org.firstinspires.ftc.teamcode.auto.base.drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auto.cv.CoreDetection;
import org.firstinspires.ftc.teamcode.auto.cv.WarehouseDetection;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.positioning.MathConstHead;
import org.firstinspires.ftc.teamcode.common.positioning.MathSpline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Disabled
@Autonomous (name = "Warehouse Cycles", group = "Test")
public class WarehouseCycles extends LinearOpMode {
    /* Declare OpMode members. */
    OpenCvCamera phoneCam;
    HardwareDrive robot = new HardwareDrive();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    Constants constants = new Constants();
    MathSpline mathSpline = new MathSpline();
    MathConstHead mathConstHead = new MathConstHead();
    WarehouseDetection tomHolland;
    int MAX_VELOCITY_DT = 2700;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    AutoHub dispatch;

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = (96.0 / 25.4);     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException{

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        dispatch = new AutoHub(this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            WarehouseDetection peterPiper = new WarehouseDetection(telemetry);
            phoneCam.setPipeline(peterPiper);

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
            //Predictions for Tomdaya breaking up
            telemetry.addLine("Camera Initiated");
            telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        dispatch.spinIntake(1);
        dispatch.constantHeading(0.75,0,22,0.001,0,0.0003);
        dispatch.turnAbsPID(-180,2);
        dispatch.spinIntake(0);
        dispatch.constantHeading(1,-20,0,0.001,0,0.0003);
        dispatch.moveElevator(constants.elevatorPositionTop);
        dispatch.constantHeading(0.75,-5,17,0.001,0,0.0003);
        dispatch.variableHeading(0.75,27,13,2);
        dispatch.spinIntake(-1,2000);
        dispatch.moveElevator(constants.elevatorPositionDown);
        dispatch.variableHeading(0.75,18 ,-12,2);
        dispatch.constantHeading(0.75,0,-20,0.001,0,0.0003);
        dispatch.turnAbsPID(0,2);

        /*
        //Start Movement
        switch (peterPiper.getLocation()){
            case ONE:
                telemetry.addData("Location", 1);
                telemetry.update();

                dispatch.constantHeading(.3, 4, 0,0.001,0,0.0003);
                dispatch.spinIntake(1);
                dispatch.constantHeading(.3,0 , 24,0.001,0,0.0003);
                dispatch.constantHeading(.3,-8, 0,0.001,0,0.0003);
                dispatch.variableHeading(.1, -2, 1, 1);
                dispatch.variableHeading(.1, 4, 1, 1);
                dispatch.spinIntake(0);
                dispatch.spinIntake(-0.4, 400);
                dispatch.constantHeading(.3, 0, -8,0.001,0,0.0003);
                dispatch.turnAbsPID(0, .5);
                dispatch.constantHeading(.3, 5, 0,0.001,0,0.0003);
                dispatch.constantHeading(.3, 0, -37,0.001,0,0.0003);
                dispatch.constantHeading(.3, -4, 0,0.001,0,0.0003);
                dispatch.turnAbsPID(-90, .5);
                dispatch.constantHeading(.5, 0, 0,0.001,0,0.0003);

                //Out-take
                dispatch.moveElevator(constants.elevatorPositionTop);
                sleep(250);
                dispatch.turnAbsPID(-90, 1);
                dispatch.constantHeading(0.5, -13, 9.5, 0.001,0,0.0003);
                dispatch.spinIntake(-.75, 2000);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.5, 0, -20, 0.001,0,0.0003);
                dispatch.turnAbsPID(0, 1);
                dispatch.constantHeading(0.5, 0, 15, 0.001,0,0.0003);

                break;

            case TWO:
                telemetry.addData("Location", 2);
                telemetry.update();
                dispatch.constantHeading(.3, 4, 0,0.001,0,0.0003);
                dispatch.spinIntake(1);
                dispatch.constantHeading(.3,0 , 24,0.001,0,0.0003);
                dispatch.constantHeading(.3,-4, 0,0.001,0,0.0003);
                dispatch.variableHeading(.1, -2, 1, 1);
                dispatch.variableHeading(.1, 4, 1, 1);
                dispatch.constantHeading(.3, 0, -8,0.001,0,0.0003);
                dispatch.spinIntake(-0.4, 400);
                dispatch.turnAbsPID(0, .5);
                dispatch.constantHeading(.3, 5, 0,0.001,0,0.0003);
                dispatch.constantHeading(.3, 0, -37,0.001,0,0.0003);
                dispatch.constantHeading(.3, -4, 0,0.001,0,0.0003);
                dispatch.turnAbsPID(-90, .5);
                dispatch.constantHeading(.5, 0, 0,0.001,0,0.0003);

                //Out-take
                dispatch.moveElevator(constants.elevatorPositionTop);
                sleep(250);
                dispatch.turnAbsPID(-90, 1);
                dispatch.constantHeading(0.5, -13, 9.5, 0.001,0,0.0003);
                dispatch.spinIntake(-.75, 2000);
                dispatch.moveElevator(constants.elevatorPositionDown);
                dispatch.constantHeading(0.5, 0, -20, 0.001,0,0.0003);
                dispatch.turnAbsPID(0, 1);
                dispatch.constantHeading(0.5, 0, 15, 0.001,0,0.0003);

                break;

            case FOUR:
            telemetry.addData("Location", 4);
            telemetry.update();

            case THREE:
                for (int i = 0; i < 2; i ++) {
                    telemetry.addData("Location", 3);
                    telemetry.update();

                    dispatch.constantHeading(0.75, 4, 0, 0.001, 0, 0.0003);
                    dispatch.spinIntake(1);
                    //yPose 24
                    dispatch.constantHeading(0.75, 0, 22, 0.001, 0, 0.0003);
                    dispatch.turn(-5);
                    dispatch.turn(5);
                    dispatch.turnAbsPID(0, 2);
                    dispatch.constantHeading(0.5, 7, 0, 0.001, 0, 0.0003);
                    dispatch.spinIntake(-0.1);
                    dispatch.moveElevator(constants.elevatorPositionTop);
                    dispatch.turnAbsPID(0, .5);
                    dispatch.spinIntake(0);
                    dispatch.constantHeading(0.75, 5, 0, 0.001, 0, 0.0003);
                    dispatch.constantHeading(0.75, 0, -46, 0.001, 0, 0.0003);
                    dispatch.constantHeading(0.75, -4, 0, 0.001, 0, 0.0003);
                    dispatch.turnAbsPID(-90, .5);
                    dispatch.constantHeading(0.75, 0, 0, 0.001, 0, 0.0003);

                    //Out-take
                    dispatch.turnAbsPID(-90, 1);
                    dispatch.constantHeading(0.75, -10, 8, 0.001, 0, 0.0003);
                    dispatch.constantHeading(0.75, 0, 3, 0.001, 0, 0.0003);
                    dispatch.turnAbsPID(-90, 1);
                    dispatch.constantHeading(0.75, -3, 6, 0.001, 0, 0.0003);
                    dispatch.spinIntake(-1, 2000);
                    dispatch.moveElevator(constants.elevatorPositionDown);
                    dispatch.constantHeading(0.75, 0, -5, 0.01, 0, 0.0003);
                    dispatch.turnAbsPID(0, 1);
                    dispatch.constantHeading(0.75, 18, 0, 0.01, 0, 0.0003);

                    dispatch.turnAbsPID(0, 1);
                    dispatch.constantHeading(0.5, 0, 37, 0.01, 0, 0.0003);
                }

                break;
            case SPIDER_MAN_HOME_PNG_DOES_NOT_EXIST:
                telemetry.addData("Location", "Tom Holland Homeless");
                telemetry.update();
                break;
        }

         */

        //End of Path
        telemetry.update();
    }
}
