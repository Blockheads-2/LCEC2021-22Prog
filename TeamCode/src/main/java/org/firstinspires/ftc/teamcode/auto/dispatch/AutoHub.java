package org.firstinspires.ftc.teamcode.auto.dispatch;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.Date;
import java.util.List;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;
import javax.xml.xpath.XPathExpressionException;

public class AutoHub {

    private static final String TAG = "FTCAuto";

    private final LinearOpMode linearOpMode;
    private final HardwareDrive robot = new HardwareDrive();
    private double desiredHeading = 0.0; // always normalized

    public AutoHub(LinearOpMode plinear){
        linearOpMode = plinear;
    }

    public void doCommand(String command){
        switch (command) {
            case "STRAIGHT": {
                robot.lf.setPower(0.5);
                robot.rf.setPower(0.5);
                robot.lb.setPower(0.5);
                robot.rb.setPower(0.5);

                sleep(1000);

                robot.lf.setPower(0);
                robot.rf.setPower(0);
                robot.lb.setPower(0);
                robot.rb.setPower(0);
            }
        }
    }

}

