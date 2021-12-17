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
        robot.init(hardwareMap);

        waitForStart();
        dispatch.variableHeading(0.5,20,20,2);
    }
}