package org.firstinspires.ftc.teamcode.hardware;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SamplePipeline extends OpenCvPipeline {

    final Scalar BLUE = new Scalar(0, 0, 255);
    final Scalar GREEN = new Scalar(0, 255, 0);

    int detectionThreshold = 125;

    Point r1_point1 = new Point(50, 120);
    Point r1_point2 = new Point(150, 240);

    Point r2_point1 = new Point(230, 120);
    Point r2_point2 = new Point(320, 240);

    Mat region1_Cr, region2_Cr;
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    static int avg1;
    int avg2;

    void inputToCr(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCr(firstFrame);

        region1_Cr = Cr.submat(new Rect(r1_point1, r1_point2));
        region2_Cr = Cr.submat(new Rect(r2_point1, r2_point2));
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(input, r1_point1, r1_point2, BLUE, 2);
        Imgproc.rectangle(input, r2_point1, r2_point2, BLUE, 2);
        inputToCr(input);

        avg1 = (int) Core.mean(region1_Cr).val[0];
        avg2 = (int) Core.mean(region2_Cr).val[0];

        if (inRegion1()) {
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    r1_point1, // First point which defines the rectangle
                    r1_point2, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1);
        } else if (inRegion2()) {
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    r2_point1, // First point which defines the rectangle
                    r2_point2, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1);
        }
        return input;
    }

    public int getAvg1() {
        return avg1;
    }

    public int getAvg2() {
        return avg2;
    }

    public boolean inRegion1() {
        return getAvg1()<detectionThreshold;
    }

    public boolean inRegion2() {
        return avg2<detectionThreshold;
    }
}