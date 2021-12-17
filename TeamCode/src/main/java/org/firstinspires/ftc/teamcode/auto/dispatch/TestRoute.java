package org.firstinspires.ftc.teamcode.auto.dispatch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;


@Autonomous(name = "Test Route", group = "routes")
//@Disabled
public class TestRoute extends LinearOpMode {
    AutoHub dispatch;
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

        dispatch.doCommand("STRAIGHT");
    }
}