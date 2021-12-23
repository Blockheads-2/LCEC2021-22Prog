package org.firstinspires.ftc.teamcode.auto.cv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class WarehouseDetection extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        ONE,
        TWO,
        THREE,
        FOUR,
        SPIDER_MAN_HOME_PNG_DOES_NOT_EXIST
    }
    private Location location;

    /*
    static final Rect RIGHT_ROI = new Rect(
            new Point(50, 180),
            new Point(250, 420));
    */

    static final Rect ROI_1 = new Rect(
            new Point(50,400),
            new Point(150,600));
    static final Rect ROI_2 = new Rect(
            new Point(200,400),
            new Point(350,600));
    static final Rect ROI_3 = new Rect(
            new Point(400, 400),
            new Point(550, 600));
    static final Rect ROI_4 = new Rect(
            new Point(600,400),
            new Point(750,600));


    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public WarehouseDetection (Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(20, 50, 70);
        Scalar highHSV = new Scalar(35, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat roi1 = mat.submat(ROI_1);
        Mat roi2 = mat.submat(ROI_2);
        Mat roi3 = mat.submat(ROI_3);
        Mat roi4 = mat.submat(ROI_4);

        double roi1Value = Core.sumElems(roi1).val[0] / ROI_1.area() / 255;
        double roi2Value = Core.sumElems(roi2).val[0] / ROI_2.area() / 255;
        double roi3Value = Core.sumElems(roi3).val[0] / ROI_3.area() / 255;
        double roi4Value = Core.sumElems(roi4).val[0] / ROI_4.area() / 255;


        roi1.release();
        roi2.release();
        roi3.release();
        roi4.release();


        boolean cube1 = roi1Value > PERCENT_COLOR_THRESHOLD;
        boolean cube2 = roi2Value > PERCENT_COLOR_THRESHOLD;
        boolean cube3 = roi3Value > PERCENT_COLOR_THRESHOLD;
        boolean cube4 = roi4Value > PERCENT_COLOR_THRESHOLD;

        if (cube4) {
            location = Location.FOUR;
            telemetry.addData("Cube Location", "4");
        } else if (cube3) {
            location = Location.THREE;
            telemetry.addData("Cube Location", "3");
        } else if (cube2) {
            location = Location.TWO;
            telemetry.addData("Cube Location", "2");
        } else if (cube1) {
            location = Location.ONE;
            telemetry.addData("Cube Location", "1");
        } else {
            location = Location.THREE;
            telemetry.addData("Cube Location", "Not Found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorOtherStone = new Scalar(0, 0, 255);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, ROI_1, location == Location.ONE? colorSkystone:colorStone);
        Imgproc.rectangle(mat, ROI_2, location == Location.TWO? colorSkystone:colorOtherStone);
        Imgproc.rectangle(mat, ROI_3, location == Location.THREE? colorSkystone:colorOtherStone);
        Imgproc.rectangle(mat, ROI_4, location == Location.FOUR? colorSkystone:colorOtherStone);
        Imgproc.rectangle(mat, ROI_4, location == Location.SPIDER_MAN_HOME_PNG_DOES_NOT_EXIST? colorSkystone:colorOtherStone);
        return mat;
    }

    public Location getLocation() {
        return location;
    }
}