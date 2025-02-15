package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Robot.ad;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="Left Three W Turning", preselectTeleOp="Telop")
public class LeftThreeWTurning extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, true);

        //Prep Robot for Auto
        outtake.stopVSlide();
        outtake.setBucketPos(outtake.bucketRegPos);
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();

        waitForStart();

        goAndScore(true, true, true);

        goAndIntakeAndTransfer(2);

        goAndScore(true, true, true);
        goAndIntakeAndTransfer(3);


        goAndScore(true, true, true);

        goAndIntakeAndTransfer(1);


        goAndScore(true, true, false);

        sleep(3000);
    }
    //Method Drives Robot to Bucket and Dumps Sample
    public void goAndScore(boolean high, boolean goToPos, boolean extendHSlide){
        //Going to Bucket with Sample
        if(high){
            outtake.vslideToPos(outtake.highBucketSlidePos, outtake.slidePower);
        } else {
            outtake.vslideToPos(outtake.lowBucketSlidePos, outtake.slidePower);
        }
        if(extendHSlide) {
            intake.hslideToPos(intake.slideOut - 600, .5);
        }

        if(goToPos) {
            ad.goToHeading(0);
            ad.goToPointConstantHeading(13, 12);
        }
        ad.goToHeading(315);
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
        outtake.vslideToPos(outtake.bottomSlidePos, outtake.slidePower);

    }

    //Robot Collects Inputted Sample (Left = 1, Middle = 2, Right = 3)
    public void goAndIntakeAndTransfer(int sample){
        //
        double angle = 0;
        switch (sample) {
            case 1:
                angle = 20; // 20
                break;
            case 2:
                angle = 1; //5
                break;
            case 3:
                angle = 344;//346
                break;
        }
        intake.runWheels(true);
        //Start HSlide Slowly
        intake.tsTarget = .325;//intake.tsDown;
        intake.setTransferServo();


        //allows us to start moving the transfer servo mid movement to save time


        ad.goToHeading(angle);
        sleep(250);
        if(sample == 2 || sample == 3){
            sleep(200);
        }
        if(sample == 3){
            sleep(100);
        }
        intake.tsTarget = intake.tsDown;
        intake.setTransferServo();

        if(sample == 2) sleep(350);
        intake.hslideToPos(intake.slideOut+100, 1);



        //Turn Active Intake On
        //while(!intake.hasSample()){}

        sleep(400);
        if(sample == 1 || sample == 3){
            sleep(150);
        }

        //Move Transfer Servo to Middle
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();
        intake.stopWheels();

        intake.hslideToPos(intake.slideForceIn, 1);
        sleep(1000);

        //Transfer Sample
        intake.runWheels(true);

        sleep(800);


        //intake.stopWheels();
        intake.hslideToPow(0);



    }
}
