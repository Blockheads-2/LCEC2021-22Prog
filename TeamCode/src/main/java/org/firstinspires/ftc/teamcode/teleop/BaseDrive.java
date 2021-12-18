/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.pid.CarouselPIDController;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Base Drive", group="Drive")
//@Disabled
public class BaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    private ElapsedTime runtime = new ElapsedTime();




    Button capUpButton = new Button();
    Button capDropButton = new Button();
    Button capIntakeButton = new Button();
    Button setCapMode = new Button();
    Button carouselButton = new Button();
    Button carouselButtonInverted = new Button();
    Button lifterButton = new Button();
    Button lifterBottomButton = new Button();

    int countSmile = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
    }

    @Override
    public void init_loop() {
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }
    void UpdatePlayer1(){
        double drivePower = DriveTrainSpeed();

        DriveTrainBase(drivePower);
        DriveTrainSpeed();
        Capping();
        DriveMicroAdjust(0.4);
      //  OscillateServo();
    }

    void UpdatePlayer2(){
        Carousel();
        Lifter();
        SpinIntake();
    }

    void UpdateTelemetry(){

        telemetry.addData("lifter position", robot.lifter.getCurrentPosition());
        telemetry.addData("Carousel Velocity", robot.duckWheel.getVelocity());
      //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
        telemetry.update();
    }

    void UpdateButton(){
        capDropButton.update(gamepad1.b);
        capIntakeButton.update(gamepad1.a);
        capUpButton.update(gamepad1.y);
        setCapMode.update(gamepad1.x);


        carouselButton.update(gamepad2.a);
        carouselButtonInverted.update(gamepad2.b);
        lifterButton.update(gamepad2.y);
        lifterBottomButton.update(gamepad2.x);
    }

    /*

        void OscillateServo(){
        if (runtime.seconds() > 1){
            if (countSmile % 2 == 0)
                robot.cap.setPosition(constants.capPickUp);
            else
                robot.cap.setPosition(constants.capStart);
            runtime.reset();
            countSmile += 1;
        }
    }

     */

    void DriveTrainBase(double drivePower){
        double directionX = Math.pow(gamepad1.left_stick_x, 1); //Strafe
        double directionY = -Math.pow(gamepad1.left_stick_y, 1); //Forward
        double directionR = Math.pow(gamepad1.right_stick_x, 1); //Turn


        robot.lf.setPower((directionY + directionR + directionX) * drivePower);
        robot.rf.setPower((directionY - directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR - directionX) * drivePower);
        robot.rb.setPower((directionY - directionR + directionX) * drivePower);

    }

    void DriveMicroAdjust(double power){
        if (gamepad1.dpad_up){
            robot.lf.setPower(power);
            robot.rf.setPower(power);
            robot.lb.setPower(power);
            robot.rb.setPower(power);
        }
        else if (gamepad1.dpad_down){
            robot.lf.setPower(-power);
            robot.rf.setPower(-power);
            robot.lb.setPower(-power);
            robot.rb.setPower(-power);
        }
        else if (gamepad1.dpad_right){
            robot.lf.setPower(power);
            robot.rf.setPower(-power);
            robot.lb.setPower(-power);
            robot.rb.setPower(power);
        }
        else if (gamepad1.dpad_left){
            robot.lf.setPower(-power);
            robot.rf.setPower(power);
            robot.lb.setPower(power);
            robot.rb.setPower(-power);
        }

        if (gamepad1.left_trigger == 1){
            robot.lf.setPower(-power);
            robot.rf.setPower(power);
            robot.lb.setPower(-power);
            robot.rb.setPower(power);
        }
        else if (gamepad1.right_trigger == 1){
            robot.lf.setPower(power);
            robot.rf.setPower(-power);
            robot.lb.setPower(power);
            robot.rb.setPower(-power);
        }
    }

    double DriveTrainSpeed(){
        double drivePower = 0.75;

        if (gamepad1.right_bumper)
            drivePower = 1;
        else if (gamepad1.left_bumper)
            drivePower = 0.25;
        return drivePower;
    }

    void Lifter() {
        int position = robot.lifter.getCurrentPosition();
        if (lifterButton.is(Button.State.TAP)) {
            if (position >= (constants.elevatorPositionTop - 10)) {
                robot.lifter.setTargetPosition(constants.elevatorPositionDown);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(1);

            } else {
                robot.lifter.setTargetPosition(constants.elevatorPositionTop);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(1);
            }
        }

        /*

        if (robot.digitalTouch.getState() == false) {
            //Stop
            robot.lifter.setPower(0);

            //Reset
            robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.lifter.setTargetPosition(5);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(0.1);
        }

         */

        if (lifterBottomButton.is(Button.State.TAP)){
                if (position >= (constants.elevatorPositionBottom - 10)) {
                    robot.lifter.setTargetPosition(constants.elevatorPositionDown);
                    robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lifter.setPower(1);
                }
                else {
                    robot.lifter.setTargetPosition(constants.elevatorPositionBottom);
                    robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lifter.setPower(1);
                }
        }

        if (lifterBottomButton.is(Button.State.DOUBLE_TAP)){
            robot.lifter.setTargetPosition(constants.elevatorAcrossDrop);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(1);
        }
        if (gamepad2.left_bumper) {
            robot.lifter.setTargetPosition(position - 50);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(0.25);
        }
        if (gamepad2.right_bumper) {
            robot.lifter.setTargetPosition(position + 50);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(-0.25);
        }
    }

    void Carousel(){
        CarouselPIDController carouselPIDControllerPositive = new CarouselPIDController(1700,0.01,0,0.003);
        CarouselPIDController carouselPIDControllerNegative = new CarouselPIDController(-1700,0.01,0,0.003);

        if (carouselButton.is(Button.State.HELD)) {
            robot.duckWheel.setVelocity(1700 + carouselPIDControllerPositive.update(robot.duckWheel.getVelocity()));
        }
        else if (carouselButtonInverted.is(Button.State.HELD)) {
            robot.duckWheel.setVelocity(-1700 + carouselPIDControllerNegative.update(robot.duckWheel.getVelocity()));
        }
        robot.duckWheel.setPower(0);
    }

    void Capping(){
        if (capIntakeButton.is(Button.State.TAP))
            robot.cap.setPosition(constants.capPickUp);
        if (capUpButton.is(Button.State.TAP))
            robot.cap.setPosition(constants.capStart);
        if (capDropButton.is(Button.State.TAP))
            robot.cap.setPosition(constants.capAlmostDrop);
        if (capDropButton.is(Button.State.DOUBLE_TAP))
            robot.cap.setPosition(constants.capDrop);

    }

    void SpinIntake(){
        //Turn On
        if (gamepad2.dpad_down) // Spin In
            robot.spin.setPower(1);
        else if (gamepad2.left_trigger == 1) //Spin Out Slow
            robot.spin.setPower(-0.2);
        else if (gamepad2.dpad_up) // Spin Out Med
            robot.spin.setPower(-0.25);
        else if (gamepad2.right_trigger == 1) //Spin Out Fast
            robot.spin.setPower(0.3);
        else
            robot.spin.setPower(0);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
