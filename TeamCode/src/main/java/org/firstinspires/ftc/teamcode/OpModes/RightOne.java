package org.firstinspires.ftc.teamcode.OpModes;


import static org.firstinspires.ftc.teamcode.Robot.ad;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Right - Connor", preselectTeleOp="Telop")

public class RightOne extends LinearOpMode  {


    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, false);
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();

        //Camera Stuff
        /*CameraPipeline cam;

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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }


        });

         */




        //Prep Robot for Auto
        outtake.stopVSlide();
        outtake.setBucketPos(outtake.bucketRegPos);
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();

        outtake.closeClaw();



        waitForStart();


////////Program start////////////////////////////////////////////////////////////////////////

        //Score Preload
        goAndScore(this);
        ad.goToPointConstantHeading(24,110);

        //Move Samples to Human Player
        intakeSamples(this);
        ad.goToPointConstantHeading(10,110);
        intake.runWheels(false);
        sleep(1000);
        intake.stopWheels();
    }
    //Move to Sub and Clip Sample
    public static void goAndScore(LinearOpMode opMode){

        ad.goToHeading(180); //are heading and go to point combinable?
        outtake.vslideToPos(outtake.lowBucketSlidePos, 1);
        opMode.sleep(500);
        ad.goToPointConstantHeading(24, 72);
        opMode.sleep(500);
        outtake.killClaw();
        outtake.vslideToPos(outtake.bottomSlidePos, 1);
        opMode.sleep(1000);
    }
    //Go to Point, Extend Arm, and Sweep Samples
    public static void intakeSamples(LinearOpMode opMode){
        intake.runWheels(true);
        //Start HSlide Slowly
        intake.hslideToPos(intake.slideOut+100, .75);
        //allows us to start moving the transfer servo mid movement to save time
        intake.tsTarget = .325;//intake.tsDown;

        intake.setTransferServo();


        //Turn Active Intake On
        opMode.sleep(750);


        //Move Transfer Servo to Middle
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();
        intake.stopWheels();

        intake.hslideToPos(intake.slideForceIn, 1);
        opMode.sleep(1000);
        //intake.stopWheels();
        intake.hslideToPow(0);

        ad.goToHeading(0);





    }


}
