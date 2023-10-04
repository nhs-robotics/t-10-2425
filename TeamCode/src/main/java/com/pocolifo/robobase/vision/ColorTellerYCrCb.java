package com.pocolifo.robobase.vision;


import static com.pocolifo.robobase.vision.RegionBasedAverage.BLUE;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ColorTellerYCrCb extends AbstractResultCvPipeline {


    // Working variables. Because of memory concerns, we're not allowed to make ANY non-primitive variables within the `processFrame` method.

    //Mat is what you see
    Mat YCrCb = new Mat(), Region_Cr = new Mat(), Region_Cb = new Mat(),  Cr = new Mat(), Cb = new Mat();
    private int avg_Cr, avg_Cb, color;
    boolean went_through_init = true;
    boolean went_through_process = false;

    void inputToCr(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 1);
    }

    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    boolean check_run()
    {
        if(!went_through_process)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    int getCr() {

        return avg_Cr;
    }

    int getCb() {
        return avg_Cb;
    }

    void gridDraw(int width, int height, Mat input) {
        final Scalar GREEN = new Scalar(0, 255, 0);
        final Scalar RED = new Scalar(255,0,0);
        Scalar COLOR = GREEN;
        Point TopLeftThing = new Point(0,0);
        Point BottomRightThing = new Point(width,0);
        for (int currentHeight = 0; currentHeight < height; currentHeight += 20)
        {
            TopLeftThing.y = currentHeight;
            BottomRightThing.y = currentHeight;
            if (currentHeight % 100 == 0)
            {
                COLOR = RED;
            }
            else
            {
                COLOR = GREEN;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    TopLeftThing, // First point which defines the rectangle
                    BottomRightThing, // Second point which defines the rectangle
                    COLOR, // The color the rectangle is drawn in
                    1); // Thickness of the rectangle lines

        }
        TopLeftThing.x = 0;
        TopLeftThing.y = 0;
        BottomRightThing.x = 0;
        BottomRightThing.y = height;
        for (int currentWidth = 0; currentWidth < width; currentWidth += 20)
        {
            TopLeftThing.x = currentWidth;
            BottomRightThing.x = currentWidth;

            if (currentWidth % 100 == 0)
            {
                COLOR = RED;
            }
            else
            {
                COLOR = GREEN;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    TopLeftThing, // First point which defines the rectangle
                    BottomRightThing, // Second point which defines the rectangle
                    COLOR, // The color the rectangle is drawn in
                    1); // Thickness of the rectangle lines

        }
    }


    static final Point TopLeftAnchorPoint = new Point(300,318); //Base Picture is 600 x 480 when taken on the robot.
    static final int REGION_WIDTH = 20; //1cm
    static final int REGION_HEIGHT = 20; //1cm
    static final Point BottomRightAnchorPoint = new Point(TopLeftAnchorPoint.x + REGION_WIDTH, TopLeftAnchorPoint.y + REGION_HEIGHT);


    @Override
    public void init(Mat firstFrame) {
        inputToCr(firstFrame);
        Region_Cr = Cr.submat(new Rect(TopLeftAnchorPoint, BottomRightAnchorPoint));
        inputToCb(firstFrame);
        Region_Cb = Cb.submat(new Rect(TopLeftAnchorPoint, BottomRightAnchorPoint));
        went_through_init = true;
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCr(input);
        inputToCb(input);
        avg_Cr = (int) Core.mean(Region_Cr).val[0];
        avg_Cb = (int) Core.mean(Region_Cb).val[0];

        gridDraw(640,480,input);

        Imgproc.rectangle(
                input, // Buffer to draw on
                TopLeftAnchorPoint, // First point which defines the rectangle
                BottomRightAnchorPoint, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        went_through_process = true;
        return input;
    }

@Override
    public Integer getResult() {
        return getCr();
    }
@Override
    public Integer getSecondaryResult() {
        return getCb();
    }
}