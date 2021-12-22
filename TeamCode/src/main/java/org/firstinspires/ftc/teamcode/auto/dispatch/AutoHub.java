package org.firstinspires.ftc.teamcode.auto.dispatch;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

import org.firstinspires.ftc.teamcode.common.pid.TurnPIDController;
import org.firstinspires.ftc.teamcode.common.positioning.MathConstHead;
import org.firstinspires.ftc.teamcode.common.positioning.MathSpline;

import java.util.List;


public class AutoHub {

    private final LinearOpMode linearOpMode;


    /* Declare OpMode members. */
    HardwareDrive robot;   // Use a Pushbot's hardware
    HardwareMap hardwareMap;
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


    public AutoHub(LinearOpMode plinear){

        linearOpMode = plinear;
        hardwareMap = linearOpMode.hardwareMap;

        robot = new HardwareDrive();

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        linearOpMode.telemetry.addData("Status", "Resetting Encoders and Camera");
        linearOpMode.telemetry.update();

        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearOpMode.telemetry.addData("Status", "Ready to Run");
        linearOpMode.telemetry.update();
    }

    //====================================================================================
    // ====================================================================================

    //Core Movement
    public void variableHeading(double speed, double xPose, double yPose, double timeoutS) {
        int FleftEncoderTarget;
        int FrightEncoderTarget;
        int BleftEncoderTarget;
        int BrightEncoderTarget;

        double leftDistance;
        double rightDistance;
        double deltaTheta;
        double deltaTime;
        double zeta;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            speed = speed * MAX_VELOCITY_DT;

            mathSpline.setFinalPose(xPose,yPose);

            leftDistance = mathSpline.returnLDistance() * COUNTS_PER_INCH;
            rightDistance = mathSpline.returnRDistance() * COUNTS_PER_INCH;
            deltaTheta = mathSpline.returnTheta();
            deltaTime = leftDistance / (mathSpline.returnLPower() * constants.clicksPerInch);
            zeta = deltaTheta/deltaTime;

            double startingAngle = getAbsoluteAngle();
            double targetAngle;

            if ((yPose >= 0 && xPose < 0) || (yPose < 0 && xPose >= 0)){
                FleftEncoderTarget = robot.lf.getCurrentPosition() - (int) leftDistance;
                FrightEncoderTarget = robot.rf.getCurrentPosition() - (int) rightDistance;
                BleftEncoderTarget = robot.lb.getCurrentPosition() - (int) leftDistance;
                BrightEncoderTarget = robot.rb.getCurrentPosition() - (int) rightDistance;
            }
            else {
                FleftEncoderTarget = robot.lf.getCurrentPosition() + (int) leftDistance;
                FrightEncoderTarget = robot.rf.getCurrentPosition() + (int) rightDistance;
                BleftEncoderTarget = robot.lb.getCurrentPosition() + (int) leftDistance;
                BrightEncoderTarget = robot.rb.getCurrentPosition() + (int) rightDistance;
            }

            robot.lf.setTargetPosition(FleftEncoderTarget);
            robot.lb.setTargetPosition(BleftEncoderTarget);
            robot.rf.setTargetPosition(FrightEncoderTarget);
            robot.rb.setTargetPosition(BrightEncoderTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) && robot.lf.isBusy() && robot.rf.isBusy()
                    && robot.lb.isBusy() && robot.rb.isBusy()) {

                targetAngle = startingAngle + zeta * (runtime.milliseconds() + 1);

                TurnPIDController pidTurn = new TurnPIDController(targetAngle, 0.01, 0, 0.003);

                double angleCorrection = pidTurn.update(getAbsoluteAngle());

                robot.lf.setVelocity(speed * mathSpline.returnLPower());
                robot.rf.setVelocity(speed * mathSpline.returnRPower());
                robot.lb.setVelocity(speed * mathSpline.returnLPower());
                robot.rb.setVelocity(speed * mathSpline.returnRPower());
            }

            // Stop all motion;
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void variableHeading(double speed, double xPose, double yPose) {
        int FleftEncoderTarget;
        int FrightEncoderTarget;
        int BleftEncoderTarget;
        int BrightEncoderTarget;

        double leftDistance;
        double rightDistance;
        double deltaTheta;
        double deltaTime;
        double zeta;
        double timeoutS;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            speed = speed * MAX_VELOCITY_DT;

            mathSpline.setFinalPose(xPose,yPose);

            leftDistance = mathSpline.returnLDistance() * COUNTS_PER_INCH;
            rightDistance = mathSpline.returnRDistance() * COUNTS_PER_INCH;
            deltaTheta = mathSpline.returnTheta();
            deltaTime = leftDistance / (mathSpline.returnLPower() * constants.clicksPerInch);
            zeta = deltaTheta/deltaTime;

            double startingAngle = getAbsoluteAngle();
            double targetAngle;

            timeoutS = mathSpline.returnDistance() / (constants.clicksPerInch * speed);

            if ((yPose >= 0 && xPose < 0) || (yPose < 0 && xPose >= 0)){
                FleftEncoderTarget = robot.lf.getCurrentPosition() - (int) leftDistance;
                FrightEncoderTarget = robot.rf.getCurrentPosition() - (int) rightDistance;
                BleftEncoderTarget = robot.lb.getCurrentPosition() - (int) leftDistance;
                BrightEncoderTarget = robot.rb.getCurrentPosition() - (int) rightDistance;
            }
            else {
                FleftEncoderTarget = robot.lf.getCurrentPosition() + (int) leftDistance;
                FrightEncoderTarget = robot.rf.getCurrentPosition() + (int) rightDistance;
                BleftEncoderTarget = robot.lb.getCurrentPosition() + (int) leftDistance;
                BrightEncoderTarget = robot.rb.getCurrentPosition() + (int) rightDistance;
            }

            robot.lf.setTargetPosition(FleftEncoderTarget);
            robot.lb.setTargetPosition(BleftEncoderTarget);
            robot.rf.setTargetPosition(FrightEncoderTarget);
            robot.rb.setTargetPosition(BrightEncoderTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) && robot.lf.isBusy() && robot.rf.isBusy()
                    && robot.lb.isBusy() && robot.rb.isBusy()) {

                targetAngle = startingAngle + zeta * (runtime.milliseconds() + 1);

                TurnPIDController pidTurn = new TurnPIDController(targetAngle, 0.01, 0, 0.003);

                double angleCorrection = pidTurn.update(getAbsoluteAngle());

                robot.lf.setVelocity(speed * mathSpline.returnLPower());
                robot.rf.setVelocity(speed * mathSpline.returnRPower());
                robot.lb.setVelocity(speed * mathSpline.returnLPower());
                robot.rb.setVelocity(speed * mathSpline.returnRPower());
            }

            // Stop all motion;
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void constantHeading(double speed, double xPose, double yPose, double timeoutS, double kP, double kI, double kD) {
        mathConstHead.setFinalPose(xPose,yPose);

        double targetAngle = getAbsoluteAngle();
        TurnPIDController pidTurn = new TurnPIDController(targetAngle, kP, kI, kD);


        double distance = mathConstHead.returnDistance();
        double radianAngle = mathConstHead.returnAngle();

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        double ratioAddPose = Math.cos(radianAngle) + Math.sin(radianAngle);
        double ratioSubPose = Math.cos(radianAngle) - Math.sin(radianAngle);
        double addPose = (ratioAddPose * COUNTS_PER_INCH * distance);
        double subtractPose = (ratioSubPose * COUNTS_PER_INCH * distance);

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (robot.lf.getCurrentPosition() + addPose);
            newRightFrontTarget = (int) (robot.rf.getCurrentPosition() + subtractPose);
            newLeftBackTarget = (int) (robot.lb.getCurrentPosition() + subtractPose);
            newRightBackTarget = (int) (robot.rb.getCurrentPosition() + addPose);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {


                double angleCorrection = pidTurn.update(getAbsoluteAngle());

                robot.lf.setVelocity((speed * constants.maxVelocityDT * ratioAddPose) - (speed * angleCorrection * constants.maxVelocityDT));
                robot.rf.setVelocity((speed * constants.maxVelocityDT * ratioSubPose) + (speed * angleCorrection * constants.maxVelocityDT));
                robot.lb.setVelocity((speed * constants.maxVelocityDT * ratioSubPose) - (speed * angleCorrection * constants.maxVelocityDT));
                robot.rb.setVelocity((speed * constants.maxVelocityDT * ratioAddPose) + (speed * angleCorrection * constants.maxVelocityDT));

                // Display it for the driver.
                linearOpMode.telemetry.addData("Left Velocity: ", robot.lb.getVelocity());
                linearOpMode.telemetry.addData("Right Velocity: ", robot.rb.getVelocity());
                linearOpMode.telemetry.update();
            }

            // Stop all motion;
            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void constantHeading(double speed, double xPose, double yPose, double kP, double kI, double kD) {
        mathConstHead.setFinalPose(xPose,yPose);

        double targetAngle = getAbsoluteAngle();
        TurnPIDController pidTurn = new TurnPIDController(targetAngle, kP, kI, kD);


        double distance = mathConstHead.returnDistance();
        double radianAngle = mathConstHead.returnAngle();

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double timeoutS;

        double ratioAddPose = Math.cos(radianAngle) + Math.sin(radianAngle);
        double ratioSubPose = Math.cos(radianAngle) - Math.sin(radianAngle);
        double addPose = (ratioAddPose * COUNTS_PER_INCH * distance);
        double subtractPose = (ratioSubPose * COUNTS_PER_INCH * distance);

        timeoutS = distance / (speed * constants.clicksPerInch);

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (robot.lf.getCurrentPosition() + addPose);
            newRightFrontTarget = (int) (robot.rf.getCurrentPosition() + subtractPose);
            newLeftBackTarget = (int) (robot.lb.getCurrentPosition() + subtractPose);
            newRightBackTarget = (int) (robot.rb.getCurrentPosition() + addPose);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {


                double angleCorrection = pidTurn.update(getAbsoluteAngle());

                robot.lf.setVelocity((speed * constants.maxVelocityDT * ratioAddPose) - (speed * angleCorrection * constants.maxVelocityDT));
                robot.rf.setVelocity((speed * constants.maxVelocityDT * ratioSubPose) + (speed * angleCorrection * constants.maxVelocityDT));
                robot.lb.setVelocity((speed * constants.maxVelocityDT * ratioSubPose) - (speed * angleCorrection * constants.maxVelocityDT));
                robot.rb.setVelocity((speed * constants.maxVelocityDT * ratioAddPose) + (speed * angleCorrection * constants.maxVelocityDT));

                // Display it for the driver.
                linearOpMode.telemetry.addData("Time: ", timeoutS);
                linearOpMode.telemetry.update();
            }

            // Stop all motion;
            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //Turn
    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
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
        linearOpMode.telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }
    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (linearOpMode.opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.lf.setPower(-motorPower);
            robot.rf.setPower(motorPower);
            robot.lb.setPower(-motorPower);
            robot.rb.setPower(motorPower);

            error = degrees - getAngle();
            linearOpMode.telemetry.addData("error", error);
            linearOpMode.telemetry.update();
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
    public void turnPID(double degrees,double timeOut) {
        turnMath(-degrees + getAbsoluteAngle(), timeOut);
    }
    public void turnAbsPID(double absDegrees, double timeOut){
        turnMath(-absDegrees, timeOut);
    }
    void turnMath(double targetAngle, double timeoutS) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        linearOpMode.telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        runtime.reset();
        while ((runtime.seconds() < timeoutS) && (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75)) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.lf.setPower(-motorPower);
            robot.rf.setPower(motorPower);
            robot.lb.setPower(-motorPower);
            robot.rb.setPower(motorPower);

            linearOpMode.telemetry.addData("Current Angle", getAbsoluteAngle());
            linearOpMode.telemetry.addData("Target Angle", targetAngle);
            linearOpMode.telemetry.addData("Slope", pid.getLastSlope());
            linearOpMode.telemetry.addData("Power", motorPower);
            linearOpMode.telemetry.update();
        }
        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Peripheral Movements
    public void spinCarousel(double velocity){
        robot.duckWheel.setVelocity(velocity);
    }
    public void spinCarousel(double velocity, long spinTime){
        robot.duckWheel.setVelocity(velocity);
        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(spinTime);
        robot.duckWheel.setVelocity(0);
    }
    public void spinIntake(double power){
        robot.spin.setPower(power);
    }
    public void spinIntake(double power, long spinTime){
        robot.spin.setPower(power);
        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(spinTime);
        robot.spin.setPower(0);
    }
    public void moveElevator(int elevatorPosition){
        robot.lifter.setTargetPosition(elevatorPosition);
        robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.lifter.setPower(0.8);
    }
    public void breakPoint(){
        while (!linearOpMode.gamepad1.a) {
            sleep(1);
        }
    }

}

