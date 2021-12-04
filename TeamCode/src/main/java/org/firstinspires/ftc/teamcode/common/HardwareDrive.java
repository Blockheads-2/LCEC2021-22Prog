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

package org.firstinspires.ftc.teamcode.common;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareDrive
{
    //Motors
    public DcMotorEx  lf   = null;
    public DcMotorEx  rf   = null;
    public DcMotorEx  lb   = null;
    public DcMotorEx  rb   = null;

    public DcMotorEx duckWheel = null;
    public DcMotorEx lifter = null;

    //Servos
    public Servo cap = null;
    public CRServo spin = null;

    //Sensor
    /* --- Uncomment out when implement touch sensor --

    public DigitalChannel touchSensor;

     */

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    Constants constants = new Constants();

    /* Constructor */
    public HardwareDrive(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lf  = hwMap.get(DcMotorEx.class, "left_front");
        rf  = hwMap.get(DcMotorEx.class, "right_front");
        lb  = hwMap.get(DcMotorEx.class, "left_back");
        rb  = hwMap.get(DcMotorEx.class, "right_back");

        duckWheel = hwMap.get(DcMotorEx.class, "carousel");
        lifter = hwMap.get(DcMotorEx.class, "lifter");

        /* --- Uncomment out when implement touch sensor --
        touchSensor = hwMap.get(DigitalChannel.class, "touch");

        touchSensor.setMode(DigitalChannel.Mode.INPUT);

         */

        //Define and Initialize Servos
        cap  = hwMap.get(Servo.class, "cap");
        spin = hwMap.get(CRServo.class,"spin");

        cap.setPosition(constants.capStart);
        spin.setPower(0);


        //Reverse Motor
        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);

        duckWheel.setDirection(DcMotorEx.Direction.REVERSE);
        lifter.setDirection(DcMotorEx.Direction.REVERSE);

        // Set all motors to zero power
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        duckWheel.setPower(0);
        lifter.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        duckWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}

