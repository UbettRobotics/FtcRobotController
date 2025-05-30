package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Robot.ad;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutonomousDrive2;
import org.firstinspires.ftc.teamcode.CameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name="New Left Three W Turning", preselectTeleOp="Telop")
public class LeftThreeWTurningV2 extends LinearOpMode {

    boolean haveSample = false;



    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, true);

        AutonomousDrive2 ad2 = new AutonomousDrive2( this, true);
        ad2.setOutputInfo(true);
        ad2.setTimeLimit(4);

        CameraPipeline cam;

        int cameraMonitorViewId;

        OpenCvCamera webcam;


        cam = new CameraPipeline(telemetry);

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);

        webcam.setPipeline(cam);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            /*
             * This will be called if the camera could not be opened
             */

        });


        //Prep Robot for Auto
        outtake.stopVSlide();
        outtake.setBucketPos(outtake.bucketRegPos);
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();

        waitForStart();

        goAndScore(true, true, true, false, ad2);

        if(goAndIntakeAndTransfer(2,cam , ad2)) {
            goAndScore(true, true, true, false, ad2);
        } else {
            intake.hslideToPos(intake.slideOut - 600, .5);
        }


        if(goAndIntakeAndTransfer(3,cam, ad2)) {
            goAndScore(true, true, true, false, ad2);
        } else {
            intake.hslideToPos(intake.slideOut - 600, .5);
        }

        if(goAndIntakeAndTransfer(1,cam, ad2)){
            goAndScore(true, true, false, true, ad2);
        } else {
            intake.tsTarget = intake.tsUp;
            intake.setTransferServo();
            outtake.vslideToPos(outtake.bottomSlidePos, .75);
            outtake.setBucketPos(outtake.bucketRegPos);

            sleep(500);

            intake.hslideToPos(intake.slideForceIn, .75);

            sleep(1000);
            ad2.forward(4);

        }



        sleep(3000);
    }
    //Method Drives Robot to Bucket and Dumps Sample
    public void goAndScore(boolean high, boolean goToPos, boolean extendHSlide, boolean end, AutonomousDrive2 ad2){
        //Going to Bucket with Sample
        if(high){
            outtake.vslideToPos(outtake.highBucketSlidePos, outtake.slidePower);
        } else {
            outtake.vslideToPos(outtake.lowBucketSlidePos, outtake.slidePower);
        }


        if(goToPos) {
            ad2.goToHeading(180);
            ad2.goToPointConstantHeading(16, 13.25);
        }
        if(extendHSlide) {
            intake.hslideToPos(intake.slideOut - 700, .5);
        }
        ad2.goToHeading(315);
        intake.stopWheels();

//        if(high && !goToPos){
//            sleep(1100);
//        }
        while(Math.abs(outtake.getVSlidePos() - outtake.vslide.getTargetPosition()) > 20){

        }
        outtake.setBucketPos(outtake.bucketOutPos);
        sleep(1100);


        outtake.setBucketPos(outtake.bucketRegPos);
        sleep(400);
        if(end){
            ad2.forward(4);
        }
        outtake.vslideToPos(outtake.bottomSlidePos, .75);

    }

    //Robot Collects Inputted Sample (Left = 1, Middle = 2, Right = 3)
    public boolean goAndIntakeAndTransfer(int sample, CameraPipeline cam, AutonomousDrive2 ad2){
        //
        double angle = 0;
        switch (sample) {
            case 1:
                angle = 20; // 20
                break;
            case 2:
                angle = 3; //5
                break;
            case 3:
                angle = 345;
                break;
        }
        intake.runWheels(true);
        //Start HSlide Slowly
        intake.tsTarget = .325;//intake.tsDown;
        intake.setTransferServo();


        //allows us to start moving the transfer servo mid movement to save time


        ad2.goToHeading(180 + angle);
        sleep(250);
        if(sample == 2 || sample == 3){
            sleep(200);
        }
        if(sample == 3){
            sleep(100);
        }


        intake.tsTarget = intake.tsDown;
        intake.setTransferServo();

        if(sample == 2) sleep(450);
        else { sleep(200); }
        intake.hslideToPos(intake.slideOut+100, 1);



        //Turn Active Intake On
        //while(!intake.hasSample()){}

       /* sleep(400);
        if(sample == 1){
            sleep(150);
        }
        if(sample == 3){
            sleep(250);
        }

        */



        long startTime = System.nanoTime();

        while(!cam.isDectedted() && opModeIsActive()){
            if(getSeconds(startTime, System.nanoTime()) > 2.5){
                return false;
            }
            sleep(1);
        }





        //Move Transfer Servo to Middle
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();
        intake.stopWheels();

        intake.hslideToPos(intake.slideForceIn, 1);
        sleep(1100);

        //Transfer Sample
        intake.runWheels(true);

        sleep(500);




        //intake.stopWheels();
        intake.hslideToPow(0);

        sleep(250);

        return true;

    }

    public static double getSeconds(long startTime, long endTime){
        return (endTime - startTime) / 1000000000.0;
    }

}
