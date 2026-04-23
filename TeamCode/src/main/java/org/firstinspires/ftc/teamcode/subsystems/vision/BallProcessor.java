package org.firstinspires.ftc.teamcode.subsystems.vision;


import static org.opencv.imgproc.Imgproc.COLOR_GRAY2RGBA;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.getStructuringElement;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;

//Not done yet sdadge
class BallProcessor implements VisionProcessor {

    //things for eocv sim
    public boolean seeThres = false;
    public boolean seeMorph = false;

    // kernel stuffs
    public int dilateSize1 = 5;

    public int erodeSize1 = 5;

    public int dilateSize2 = 5;

    // bounds for color detection
    public Scalar purpleUpper = new Scalar(0, 0, 0);
    public Scalar purpleLower = new Scalar(255, 255, 255);

    public Scalar greenUpper = new Scalar(0, 0, 0);
    public Scalar greenLower = new Scalar(255, 255, 255);

    //if we use the colors
    public boolean green = false, purple = false;

    //calcs util

    double height; //mm
    HashMap<String, Double> camFOV; //deg, deg
    double camAngle; //deg

    HashMap<String, Double> cameraDimensions;
    double hypotenuse;
    double distFromCenter;
    double horiz;
    double l3;

    public BallProcessor() {
        setUpVals();
    }

    void setUpVals() {

        height = 279.4; //mm, 11 in
        camFOV = new HashMap<>(); //deg, deg
        camFOV.put("x", 49.58256);
        camFOV.put("y", 38.21321);
        camAngle = 30; //deg
        cameraDimensions = new HashMap<>(); //px, px
        cameraDimensions.put("x", 640.0);
        cameraDimensions.put("y", 480.0);
        hypotenuse = height / Math.cos(Math.toRadians(90 - camAngle)); //mm
        distFromCenter = hypotenuse * Math.tan(Math.toRadians(camFOV.get("y") / 2)); //mm
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat ground = frame.submat(new Rect(300, 400, 120, 40));
        Mat ground_hsv = new Mat();

        Imgproc.cvtColor(ground, ground_hsv, COLOR_RGB2HSV);

        double[] groundAvg = Core.mean(ground_hsv).val;

        Imgproc.putText(frame, String.format(Locale.US,"avg Color: %f.2, %f.2, %f.2",
                        groundAvg[0],
                        groundAvg[1],
                        groundAvg[2]),
                new Point(200, 300),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.5, new Scalar(255, 255, 255), 1, Imgproc.LINE_AA);
        Imgproc.rectangle(frame, new Rect(300, 400, 120, 40), Core.mean(ground), -1);
        Imgproc.rectangle(frame, new Rect(300, 400, 120, 40), new Scalar(255, 255, 255), 1);

        //TODO: auto tune :)
        /* archive from last year
            redLower = new Scalar(150,5*groundAvg[1]/3+30,groundAvg[2]*0.9);
            redUpper = new Scalar(groundAvg[0]/10+7,255,255);

            blueLower = new Scalar(groundAvg[0]*3/4+37.5,groundAvg[1]/3-25+groundAvg[0],10);
            blueUpper = new Scalar(131,255,-2.5*groundAvg[2]+80+10*groundAvg[0]);

            yellowLower = new Scalar(14,100,groundAvg[2]+10);
            yellowUpper = new Scalar(groundAvg[0]/2+15,255,255);
    */
        Mat img_hsv = new Mat();
        Imgproc.cvtColor(frame, img_hsv, COLOR_RGB2HSV);


        //NOTE: mat thresholding
        Mat img_threshold = Mat.zeros(img_hsv.rows(), img_hsv.cols(), CvType.CV_8UC1); //default, blank, idk?
        if (green) {
            Mat greenMat = new Mat();
            Core.inRange(img_hsv, greenLower, greenUpper, greenMat);
            Core.bitwise_or(img_threshold, greenMat, img_threshold);
        }
        if (purple) {
            Mat purpleMat = new Mat();
            Core.inRange(img_hsv, purpleLower, purpleUpper, purpleMat);
            Core.bitwise_or(img_threshold, purpleMat, img_threshold);
        }


        //NOTE: morphology
        //TODO: when done with testing then make these only create once

        // Fill in holes because they are whiffle balls
        Mat dilateKernel1 = getStructuringElement(MORPH_RECT, new org.opencv.core.Size(
                        2 * dilateSize1 + 1,
                        2 * dilateSize1 + 1),
                new Point(dilateSize1, dilateSize1));

        //erode for error
        Mat erodeKernel1 = getStructuringElement(MORPH_RECT, new org.opencv.core.Size(
                        2 * erodeSize1 + 1,
                        2 * erodeSize1 + 1),
                new Point(erodeSize1, erodeSize1));

        //counteract erosion
        Mat dilateKernel2 = getStructuringElement(MORPH_RECT, new org.opencv.core.Size(
                        2 * dilateSize2 + 1,
                        2 * dilateSize2 + 1),
                new Point(dilateSize2, dilateSize2));
        Mat img_morph = new Mat();

        Imgproc.dilate(img_morph, img_morph, dilateKernel1);
        Imgproc.erode(img_morph, img_morph, erodeKernel1);
        Imgproc.dilate(img_morph, img_morph, dilateKernel2);

        //NOTE: contours
        Mat hierarchy = new Mat();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(img_morph, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double bestSize = -1;
        Mat bestMat = new Mat();
        for (Mat mat : contours) {
            Rect boundRect = Imgproc.boundingRect(mat);
            if (boundRect.area() > bestSize) {
                bestSize = boundRect.area();
                bestMat = mat;
            }

        }

        Rect bestRect = Imgproc.boundingRect(bestMat);
        if (bestSize != -1) {
            Imgproc.rectangle(frame, bestRect, new Scalar(255, 0, 0), 2);
            double pixelX = bestRect.x + bestRect.width / 2.0;
            double pixelY = bestRect.y + bestRect.height / 2.0;
            double l1 = (pixelY * distFromCenter) / (cameraDimensions.get("x") / 2);
            double a1 = Math.toDegrees(Math.atan(l1 / hypotenuse));
            double a2 = (90 - camAngle) - a1;
            horiz = Math.tan(Math.toRadians(a2)) * height;

            double imageCenterX = cameraDimensions.get("x") / 2.0;
            double l2 = Math.tan(Math.toRadians(camFOV.get("x") / 2)) * horiz;
            l3 = Math.tan(Math.toRadians(((pixelX - (cameraDimensions.get("x") / 2.0)) / (cameraDimensions.get("x") / 2.0)) * (camFOV.get("x") / 2.0))) * horiz;


            Imgproc.putText(frame, String.format("(Δx,Δy): %s, %s", l3, horiz),
                    new Point(pixelX + 20, pixelY - 60),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    1, new Scalar(255, 255, 255), 2, Imgproc.LINE_AA);
        }
        if (seeThres) {
            Imgproc.cvtColor(img_threshold, img_threshold, COLOR_GRAY2RGBA);
            Core.addWeighted(frame, 0.6, img_threshold, 2, 0.1, frame);

        }
        if (seeMorph)
            img_morph.copyTo(frame);


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public double deltaX(){return l3;}
    public double getAngle(){
    return Math.atan(l3/horiz);
    }
    public double getDistance(){
        return Math.sqrt(l3*l3 + horiz * horiz);
    }
}
