package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutonomousDrive2;
import org.firstinspires.ftc.teamcode.AutonomousDrive2.*;
import org.firstinspires.ftc.teamcode.CameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "RightThree" , preselectTeleOp="Telop")
public class RightThree extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDrive2 ad2 = new AutonomousDrive2(this, true);
        ad2.setTimeLimit(2.5);
        ad2.setTimeLimit2(5);
        ad2.setOutputInfo(true);

        initAccessories(this);


        telemetry.setAutoClear(true);
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
        intake.tsTarget = intake.tsMiddle - 0.1;
        intake.setTransferServo();

        telemetry.addData("Pos X: ", ad2.getX());
        telemetry.addData("Pos Y: ", ad2.getY());
        telemetry.update();

        waitForStart();

        outtake.vslideToPos(outtake.lowBucketSlidePos, 1);
        sleep(400);
        ad2.goToPointConstantHeading(39.5,72);
        outtake.vslideToPos(0,1);
        sleep(100);

        ad2.goToPointLinear(18,120,180);

        intake.hslideToPos(2200, 1);
        sleep(200);
        intake.runWheels(true);
        intake.tsTarget = intake.tsDown;
        intake.setTransferServo();
        transferandDump(cam);

       /* ad2.goToPointConstantHeading(18,130);

        intake.hslideToPos(2200, 1);
        sleep(100);
        intake.runWheels(true);
        intake.tsTarget = intake.tsDown;
        intake.setTransferServo();
        transferandDump(cam);

        */


        outtake.vslideToPos(outtake.touchBarSlidePos, 1);
        outtake.openClaw();
        ad2.goToPointConstantHeading(7.5,119);
        outtake.closeClaw();

        goClip(ad2 , 74);

        goToWall(ad2);

        goClip(ad2, 76);

        goToWall(ad2);



    }

    public void transferandDump( CameraPipeline cam){


        long startTime = System.nanoTime();
        boolean timeOut = false;
        while(!cam.isDectedted() && opModeIsActive() && !timeOut){
            if(getSeconds(startTime, System.nanoTime()) > 2.5){
                timeOut = true;
            }
            sleep(1);
        }

        intake.stopWheels();
        intake.tsTarget = intake.tsUp;
        intake.setTransferServo();

        intake.hslideToPos(intake.slideForceIn, 1);

        while(intake.hslide.getCurrentPosition() > 100){
            sleep(1);
        }

        intake.runWheels(true);
        sleep(750);
        intake.stopWheels();
        intake.tsTarget = intake.tsMiddle-0.2;
        intake.setTransferServo();
        sleep(50);
        outtake.setBucketPos(outtake.bucketOutPos);
        sleep(1000);
        outtake.setBucketPos(outtake.bucketRegPos);


    }

    public void goClip(AutonomousDrive2 ad2, double Y){
        outtake.vslideToPos(outtake.lowBucketSlidePos,1);
        ad2.goToPointLinear(36,Y,0);
        ad2.goToPointConstantHeading(39.5,Y);
        outtake.vslideToPos(0,1);
        sleep(100);
    }

    public void goToWall(AutonomousDrive2 ad2) {
        outtake.vslideToPos(outtake.touchBarSlidePos, 1);
        outtake.openClaw();
        ad2.goToPointLinear(7.5,119, 180);
        ad2.goToPointConstantHeading(8,120);
        outtake.closeClaw();
    }
    public static double getSeconds(long startTime, long endTime){
        return (endTime - startTime) / 1000000000.0;
    }

}
