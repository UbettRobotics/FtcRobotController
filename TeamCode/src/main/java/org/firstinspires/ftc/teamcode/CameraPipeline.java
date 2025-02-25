
package org.firstinspires.ftc.teamcode;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;

public class CameraPipeline extends OpenCvPipeline {

    // scalar is in order of hue, saturation, value (hsv)
    public final Scalar[] redBounds = {new Scalar(0, 182, 163), new Scalar(18, 255, 255)};
    public final Scalar[] blueBounds = {new Scalar(105, 106, 47), new Scalar(142, 255, 255)};
    public final Scalar[] neonBounds = {new Scalar(40, 120, 40), new Scalar(83, 255, 255)};


    public Scalar[] currentBounds = blueBounds;

    Telemetry telemetry;
    public Rect rectCrop = new Rect(0, 0, 320, 240);
    Mat mat = new Mat();
    Mat mat2 = new Mat();
    Date date = new Date();
    Mat image;
    Mat hierarchy;


    private static double  power;

    //color side
    int fieldColor = 1;

    int park = 1;

    final int MAX_CONTOURS = 10;//Must be higher than expected number of contours

    public enum Side {
        LEFT_SIDE,
        MIDDLE_SIDE,
        RIGHT_SIDE
    }
    public int rectX = 0;
    //This is the enum that is used throughout the pipeline (lowercase variable)
    //It is type 'Side' (as opposed to a long or boolean)
    public static Side side = Side.RIGHT_SIDE;

    //HSV color parameters, determine w/ Python live update program
    private int hueMin = 0; //84
    private int hueMax = 144; //122
    private int satMin = 100; //88
    private int satMax = 255;
    private int valMin = 163; //30
    private int valMax = 255;
    private ArrayList<Integer> coneAreaArray;
    private ArrayList<Mat> conarr = new ArrayList<Mat>();
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    private ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

    public CameraPipeline(Telemetry t) { telemetry = t;}
    //We use EasyOpenCv, but the docs for what this does will be OpenCv docs
    @Override
    public Mat processFrame(Mat input) {
        //converts frame to HSV colorspace
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);


        //the range of colors to filter.
        Scalar lowHSV = new Scalar(hueMin, satMin, valMin);
        Scalar highHSV = new Scalar(hueMax, satMax, valMax);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Imgproc.findContours(mat, contours, mat2, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        updateList(mat);

        telemetry.addData("Contours", conarr.size());
        telemetry.addData("Is detected: ", isDectedted());
        telemetry.update();


        return image;
    }





    /*private List<Integer> getContours(Mat mat){
        image = mat.clone();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();


        ArrayList<Integer> arr = new ArrayList<Integer>();
        conarr.clear();


        telemetry.update();

        double minArea = 50;
        for (int i = 0; i < contours.size(); i++) {
            Mat contour = contours.get(i);
            int contourArea = Imgproc.boundingRect(contour).y;
            //Add any contours bigger than error size (ignore tiny bits) to array of all contours
            if(contourArea > minArea){
                arr.add(contourArea);
                conarr.add(contour);
                Rect bounding = Imgproc.boundingRect(contour);
                //Draw a rectangle on preview stream
                Imgproc.rectangle(image, bounding, new Scalar(80,80,80), 4);
            }
        }


        //side = getConeArea();


        return arr;


    }

    /*public void gotoSpecimen(OpMode opMode){

    }

     */


    public boolean isDectedted(){
        return (conarr.size() > 0);
    }







    private void updateList(Mat mat) {

        hierarchy = new Mat();
        image = mat.clone();
        contours = new ArrayList<MatOfPoint>();

        Imgproc.findContours(image, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        conarr = new ArrayList<Mat>();



        double minArea = 50;
        for (int i = 0; i < contours.size(); i++) {
            Mat contour = contours.get(i);
            double contourArea = Imgproc.contourArea(contour);
            //Add any contours bigger than error size (ignore tiny bits) to array of all contours
            if(contourArea > minArea){
                conarr.add(contour);
                Rect bounding = Imgproc.boundingRect(contour);

                //Draw a rectangle on preview stream
                Imgproc.rectangle(image, bounding, new Scalar(80,80,80), 4);
            }
        }



    }






}