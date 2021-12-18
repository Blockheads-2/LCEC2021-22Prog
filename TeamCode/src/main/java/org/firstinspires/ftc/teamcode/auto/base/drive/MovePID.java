package org.firstinspires.ftc.teamcode.auto.base.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.pid.MovePIDController;
import org.firstinspires.ftc.teamcode.common.pid.TurnPIDController;


@Autonomous(name="Move PID", group="Test")
@Disabled
public class MovePID extends LinearOpMode{

    /* Declare OpMode members. */
    HardwareDrive robot   = new HardwareDrive();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    Constants constants = new Constants();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() throws InterruptedException{

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveStraight(0.5,24);

    }

    public double getAngle() {
        // Get current orientation
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.lf.setPower(-motorPower);
            robot.rf.setPower(motorPower);
            robot.lb.setPower(-motorPower);
            robot.rb.setPower(motorPower);

            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);

    }

    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    void driveStraight(double power, double distance) {
        double targetAngle = getAbsoluteAngle();
        double distanceFoward = distance * constants.clicksPerInch;

        TurnPIDController pidTurn = new TurnPIDController(targetAngle, 0.001, 0, 0.00003);

        double lfDistance = robot.lf.getCurrentPosition() + distanceFoward;
        double rfDistance = robot.rf.getCurrentPosition() + distanceFoward;
        double lbDistance = robot.lb.getCurrentPosition() + distanceFoward;
        double rbDistance = robot.rb.getCurrentPosition() + distanceFoward;

        runtime.reset();

        while (runtime.seconds() < 3) {

            robot.lf.setTargetPosition((int) lfDistance);
            robot.rf.setTargetPosition((int) rfDistance);
            robot.lb.setTargetPosition((int) lbDistance);
            robot.rb.setTargetPosition((int) rbDistance);

            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            double angleCorrection = pidTurn.update(getAbsoluteAngle());


            robot.lf.setPower(power - (power * angleCorrection));
            robot.rf.setPower(power + (power * angleCorrection));
            robot.lb.setPower(power - (power * angleCorrection));
            robot.rb.setPower(power + (power * angleCorrection));
        }
        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
    }
}