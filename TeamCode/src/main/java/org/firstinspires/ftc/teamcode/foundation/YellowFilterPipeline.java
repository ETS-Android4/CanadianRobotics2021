package org.firstinspires.ftc.teamcode.foundation;
import android.webkit.WebStorage;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

public class YellowFilterPipeline extends OpenCvPipeline {
    Scalar lower = new Scalar(22, 93, 0);
    Scalar upper = new Scalar(45, 255, 255);
    Mat OriginalFull = new Mat();
    Mat Yellow = new Mat();
    Mat MatA = new Mat(), MatB = new Mat(), MatC = new Mat();
    public int avgA, avgB, avgC;

    static final int WIDTH = 1280; // modify for your camera
    static final int HEIGHT = 720; // modify for your camera

    static final Rect RectA = new Rect(0, 0, WIDTH / 3, HEIGHT);
    static final Rect RectB = new Rect(WIDTH / 3, 0, WIDTH / 3, HEIGHT);
    static final Rect RectC = new Rect(WIDTH / 3 * 2, 0, WIDTH / 3, HEIGHT);

    public int getRect() {
        //if (avgA <= 1 && avgB <= 1 && avgC <= 1)
            //return -1; // None detected
        if (avgA > avgB && avgA > avgC)
            return 0; // In region A
        if (avgB > avgA && avgB > avgC)
            return 1; // In region B
        if (avgC > avgA && avgC > avgB)
            return 2; // In region C
        return -1; // Fallback
    }

    @Override
    public void init(Mat firstFrame) {}

    @Override
    public Mat processFrame(Mat input) {
        // Separates Y value and divides image
        Imgproc.cvtColor(input, OriginalFull, Imgproc.COLOR_RGB2HSV);
        Core.inRange(OriginalFull, lower, upper, Yellow);
        Imgproc.cvtColor(OriginalFull, input, Imgproc.COLOR_HSV2RGB);
        MatA = Yellow.submat(RectA);
        MatB = Yellow.submat(RectB);
        MatC = Yellow.submat(RectC);
        // Updates averages
        avgA = (int) Core.mean(MatA).val[0];
        avgB = (int) Core.mean(MatB).val[0];
        avgC = (int) Core.mean(MatC).val[0];
        // Prevents memory leak, errorless crash
        OriginalFull.release();
        Yellow.release();
        MatA.release();
        MatB.release();
        MatC.release();
        // DEBUG: Draw rectangles to screen
        Imgproc.rectangle(input, RectA, new Scalar(0, 0, 255), 2);
        Imgproc.rectangle(input, RectB, new Scalar(0, 0, 255), 2);
        Imgproc.rectangle(input, RectC, new Scalar(0, 0, 255), 2);
        return input;
    }
}