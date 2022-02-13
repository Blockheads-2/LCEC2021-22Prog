package org.firstinspires.ftc.teamcode.teleop.test;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestBlockDetection extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        FOUND,
        NOT_FOUND
    }
    private Location location;
    private int amountChanged = 0;

    static final Rect ROI = new Rect(
            new Point(700, 400),
            new Point(1000, 600));

    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public TestBlockDetection (Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(40 + amountChanged, 30, 42);
        Scalar highHSV = new Scalar(50 + amountChanged, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat roi = mat.submat(ROI);

        double ROI_Value = Core.sumElems(roi).val[0] / ROI.area() / 255;

        roi.release();

        boolean stoneROI = ROI_Value > PERCENT_COLOR_THRESHOLD;

        if (stoneROI) {
            location = Location.FOUND;
            telemetry.addLine("Found");
        } else {
            location = Location.NOT_FOUND;
            telemetry.addLine("Not Found");
        }

        telemetry.addData("Counter", amountChanged);

        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorDetected = new Scalar(255, 0, 0);
        Scalar colorUndetected = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, ROI, location == Location.FOUND? colorDetected:colorUndetected);

        return mat;
    }

    public void fluctuateHSV(int amountChanged) {
        this.amountChanged = amountChanged;
    }

    public Location getLocation() {
        return location;
    }
}