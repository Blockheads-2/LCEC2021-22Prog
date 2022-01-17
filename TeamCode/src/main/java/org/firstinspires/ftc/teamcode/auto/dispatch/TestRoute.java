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

        dispatch.constantHeading(1,25,0,true,0.001,0,0.0003);
        dispatch.constantHeading(0.75,9,40,true,0.001,0,0.0003);
        dispatch.variableHeading(0.75,-35,10,2);


    }
}