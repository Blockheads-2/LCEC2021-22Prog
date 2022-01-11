package org.firstinspires.ftc.teamcode.auto.dispatch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;


@Autonomous(name = "Test Route", group = "Routes")
//@Disabled
public class TestRoute extends LinearOpMode {
    AutoHub dispatch;

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

        waitForStart();

        dispatch.spinIntake(1);
        dispatch.constantHeading(0.2,0,100,true,0.001,0,0.0003);


    }
}