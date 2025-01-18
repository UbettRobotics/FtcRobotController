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

        goAndScore(true, true);

        goAndIntakeAndTransfer(2);

        goAndScore(true, false);
        goAndIntakeAndTransfer(3);


        goAndScore(true, false);

        goAndIntakeAndTransfer(1);


        goAndScore(true, false);

    }
    //Method Drives Robot to Bucket and Dumps Sample
    public void goAndScore(boolean high, boolean goToPos){
        //Going to Bucket with Sample
        if(high) {
            outtake.vslideToPos(outtake.highBucketSlidePos, outtake.slidePower);
        } else {
            outtake.vslideToPos(outtake.lowBucketSlidePos, outtake.slidePower);
        }

        if(goToPos) {
            ad.goToHeading(0);
            ad.goToPointConstantHeading(14.25, 13.5);
        }
        ad.goToHeading(315);
        intake.stopWheels();

        if(high && !goToPos){
            sleep(1100);
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
                angle = 20;
                break;
            case 2:
                angle = 5;
                break;
            case 3:
                angle = 346;
                break;
        }
        intake.runWheels(true);
        //Start HSlide Slowly
        intake.hslideToPos(intake.slideOut+100, .75);
        //allows us to start moving the transfer servo mid movement to save time
        intake.tsTarget = .325;//intake.tsDown;

        intake.setTransferServo();

        ad.goToHeadingEvent(angle, 10, ()->{intake.tsTarget = intake.tsDown; intake.setTransferServo();});
        //, 180, () -> {intake.tsTarget = intake.tsDown; intake.setTransferServo();});

        //Speed up HSlide after turn
        intake.hslideToPos(intake.slideOut+100, 1);






        //Turn Active Intake On
        if(sample == 1) {
            sleep(650);
        } else {
            sleep(650);
        }


        //Move Transfer Servo to Middle
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();
        intake.stopWheels();

        intake.hslideToPos(intake.slideForceIn, 1);
        sleep(1200);

        //Transfer Sample
        intake.runWheels(true);

        sleep(1000);


        //intake.stopWheels();
        intake.hslideToPow(0);



    }
}
