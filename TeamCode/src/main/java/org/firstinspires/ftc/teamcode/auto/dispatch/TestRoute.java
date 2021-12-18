package org.firstinspires.ftc.teamcode.auto.dispatch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;


@Autonomous(name = "Test Route", group = "routes")
//@Disabled
public class TestRoute extends LinearOpMode {
    AutoHub dispatch;
    HardwareDrive robot = new HardwareDrive();

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

        waitForStart();
        dispatch.constantHeading(0.9,-10,5,2,0.001,0,0.0003);

    }
}