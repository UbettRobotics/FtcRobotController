
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

    Mat image1 = new Mat();
    Mat image2 = new Mat();
    Mat image3 = new Mat();

    Mat image21 = new Mat();
    Mat image22 = new Mat();
    Mat image23 = new Mat();

    Date date = new Date();
    Mat image;
    Mat imageOrg;


    private static double  power;

    //color side
    int fieldColor = 1;

    int park = 1;

    final int MAX_CONTOURS = 10;//Must be higher than expected number of contours


    public int rectX = 0;
    //This is the enum that is used throughout the pipeline (lowercase variable)
    //It is type 'Side' (as opposed to a long or boolean)


    //HSV color parameters, determine w/ Python live update program
    private int hueMin = 0; //84
    private int hueMax = 144; //122
    private int satMin = 100; //88
    private int satMax = 255;
    private int valMin = 163; //30
    private int valMax = 255;



    private int hueMinRed = 12; //84
    private int hueMaxRed = 37; //122
    private int satMinRed = 113; //88
    private int satMaxRed = 255;
    private int valMinRed = 206; //30
    private int valMaxRed = 255;

    private int hueMinBlue = 105; //84
    private int hueMaxBlue = 142; //122
    private int satMinBlue = 106; //88
    private int satMaxBlue = 255;
    private int valMinBlue = 47; //30
    private int valMaxBlue = 255;

    private int hueMinYellow = 0; //84
    private int hueMaxYellow = 18; //122
    private int satMinYellow = 182; //88
    private int satMaxYellow = 255;
    private int valMinYellow = 163; //30
    private int valMaxYellow = 255;
    private ArrayList<Integer> coneAreaArray;
    private ArrayList<Mat> conarr = new ArrayList<Mat>();
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    int num1, num2, num3 = 0;

    private ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> contoursRed = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> contoursBlue = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> contoursYellow = new ArrayList<MatOfPoint>();

    public CameraPipeline(Telemetry t) { telemetry = t;}
    //We use EasyOpenCv, but the docs for what this does will be OpenCv docs
    @Override
    public Mat processFrame(Mat input) {
        //converts frame to HSV colorspace
        imageOrg = input.clone();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, image1, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, image2, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, image3, Imgproc.COLOR_RGB2HSV);


        //the range of colors to filter.
        Scalar lowHSV = new Scalar(hueMin, satMin, valMin);
        Scalar highHSV = new Scalar(hueMax, satMax, valMax);

        Scalar lowHSVRed = new Scalar(hueMinRed, satMinRed, valMinRed);
        Scalar highHSVRed = new Scalar(hueMaxRed, satMaxRed, valMaxRed);

        Scalar lowHSVBlue = new Scalar(hueMinBlue, satMinBlue, valMinBlue);
        Scalar highHSVBlue = new Scalar(hueMaxBlue, satMaxBlue, valMaxBlue);

        Scalar lowHSVYellow = new Scalar(hueMinYellow, satMinYellow, valMinYellow);
        Scalar highHSVYellow = new Scalar(hueMaxYellow, satMaxYellow, valMaxYellow);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Core.inRange(image1, lowHSVRed, highHSVRed, image1);
        Core.inRange(image2, lowHSVBlue, highHSVBlue, image2);
        Core.inRange(image3, lowHSVYellow, highHSVYellow, image3);

        Imgproc.findContours(mat, contours, mat2, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(image1, contoursRed, image21, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(image2, contoursBlue, image22, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(image3, contoursYellow, image23, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        updateList(mat);

        num1 = findContours(image1, 0).size();
        num2 = findContours(image2,1).size();
        num3 = findContours(image3,2).size();


        telemetry.addData("Contours", conarr.size());
        telemetry.addData("Is detected: ", isDectedted());
        if(conarr.size() > 0) telemetry.addData("Size: ", Imgproc.contourArea(conarr.get(0)));
        telemetry.addData("Num1: ", num1);
        telemetry.addData("Num2: ", num2);
        telemetry.addData("Num3: ", num3);

        telemetry.update();


        return imageOrg;
    }





    public boolean isDectedted(){
        return (conarr.size() > 0);
    }

    public int findColor(){
       int max = Math.max(num1,Math.max(num2,num3));
       if(max == 0){return -1;}

       /*
       0 -> Red
       1 -> Blue
       2 -> Yellow
        */
       if(max == num1){
           return 2;
       }
        if(max == num2){
            return 1;
        }
        if(max == num3){
            return 0;
        }

        return -1;
    }

    private ArrayList<Mat> findContours(Mat mat, int colorNum){

        ArrayList<MatOfPoint> newContours;
        Mat hierarchy = new Mat();
        Mat newImage = mat.clone();
        if(colorNum == 1){
            contoursBlue = new ArrayList<MatOfPoint>();
            newContours = contoursBlue;
        }else if(colorNum == 2){
            contoursYellow = new ArrayList<MatOfPoint>();
             newContours = contoursYellow;
        }else{
            contoursRed = new ArrayList<MatOfPoint>();
            newContours = contoursRed;
        }

        ArrayList<Mat> validContours =  new ArrayList<Mat>();



        Imgproc.findContours(newImage, newContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);



        double minArea = 50;
        for (int i = 0; i < newContours.size(); i++) {
            Mat contour = newContours.get(i);
            double contourArea = Imgproc.contourArea(contour);
            //Add any contours bigger than error size (ignore tiny bits) to array of all contours
            if(contourArea > minArea){
                validContours.add(contour);
                Rect bounding = Imgproc.boundingRect(contour);

                //Draw a rectangle on preview stream
                Imgproc.rectangle(imageOrg, bounding, new Scalar(80,80,80), 4);
            }
        }


        return validContours;



    }







    private void updateList(Mat mat) {

        Mat hierarchy = new Mat();
        image = mat.clone();
        contours = new ArrayList<MatOfPoint>();

        Imgproc.findContours(image, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        conarr = new ArrayList<Mat>();



        double minArea = 17400;
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