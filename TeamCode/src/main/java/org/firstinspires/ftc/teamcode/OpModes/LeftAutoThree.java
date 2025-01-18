package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Robot.ad;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Autonomous (name="Left Three Auto", preselectTeleOp="Telop")
public class LeftAutoThree extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, true);

        //Prep Robot for Auto
        outtake.stopVSlide();
        outtake.setBucketPos(outtake.bucketRegPos);
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();

        waitForStart();

        goAndScore(true);

        goAndIntakeAndTransfer(2);


        goAndScore(true);


        goAndIntakeAndTransfer(3);
        goAndScore(true);

        sleep(5000);



    }

    public void goAndScore(boolean high){
        //Going to Bucket with Sample
        if(high) {
            outtake.vslideToPos(outtake.highBucketSlidePos, outtake.slidePower);
        } else {
            outtake.vslideToPos(outtake.lowBucketSlidePos, outtake.slidePower);
        }
        ad.goToPointConstantHeading(15, 15);
        ad.goToHeading(315);

        //Dump Sample
        ad.forward(-3);
        outtake.setBucketPos(outtake.bucketOutPos);
        sleep(670);

        //Move Away from Bucket
        ad.forward(3);
        outtake.setBucketPos(outtake.bucketRegPos);
        outtake.vslideToPos(outtake.bottomSlidePos, outtake.slidePower);

    }


    public void goAndIntakeAndTransfer(int sample){ // sample is 1, 2, 3 (left to right)
        //Line Up With Floor Sample
        ad.goToHeading(0);
        ad.goToPointConstantHeading(25.5,12 + (sample-2) * 10);

        //Put Intake Down
        intake.tsTarget = intake.tsDown;
        intake.setTransferServo();

        //Turn Active Intake On
        intake.runWheels(true);
        intake.hslideToPos(1600, 1);
        sleep(1000);

        //Move Transfer Servo to Middle
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();
        intake.stopWheels();

        intake.hslideToPos(-200, 1);
        sleep(200);

        //Transfer Sample
        intake.runWheels(true);
        for(int i = 0; i < 3; i++) {
            intake.tsTarget = intake.tsUp;
            intake.setTransferServo();
            sleep(300);
            intake.tsTarget = intake.tsMiddle;
            intake.setTransferServo();
            sleep(100);
        }
        intake.stopWheels();
        intake.hslideToPow(0);



    }
}