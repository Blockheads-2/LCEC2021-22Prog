package org.firstinspires.ftc.teamcode.auto.dispatch;

import static android.os.SystemClock.sleep;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

import org.firstinspires.ftc.teamcode.common.pid.TurnPIDController;
import org.firstinspires.ftc.teamcode.common.positioning.MathConstHead;
import org.firstinspires.ftc.teamcode.common.positioning.MathSpline;


@Autonomous(name = "Test Route", group = "Test")
//@Disabled
public class TestRoute extends LinearOpMode {
    AutoHub dispatch;
    HardwareDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareDrive();
        dispatch = new AutoHub(this);

        waitForStart();

        dispatch.variableHeading(0.6,24,24,4);

    }
}
