package org.firstinspires.ftc.teamcode.OpModes;


import static org.firstinspires.ftc.teamcode.Robot.ad;
import static org.firstinspires.ftc.teamcode.Robot.huskCam;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CameraPipeline;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Right Onr W Turning", preselectTeleOp="Telop")

public class RightOneWVison extends LinearOpMode  {


    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, false);

        //Prep Robot for Auto
        outtake.stopVSlide();
        outtake.setBucketPos(outtake.bucketRegPos);
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();
        telemetry.addData("Heading: ", ad.getHeading());
        telemetry.addData("X: ", ad.getX());
        telemetry.addData("Y: ",  ad.getY());




        waitForStart();
////////Program start////////////////////////////////////////////////////////////////////////
        ad.goToHeading(180);
        outtake.vslideToPos(outtake.lowBucketSlidePos,1);
        sleep(500);
        ad.goToPointConstantHeading(40,72);
        ad.goToHeading(180);
        clip(this);
        sleep(1000);





    }

    public static void clip(LinearOpMode opMode){
        outtake.killClaw();
        outtake.vslideToPos(outtake.bottomSlidePos + 50,1);
        opMode.sleep(500);

    }
    public static void dumpBucket(LinearOpMode opMode){
        intake.tsTarget =intake.tsMiddle;
        intake.setTransferServo();
        outtake.setBucketPos(outtake.bucketOutPos);
        opMode.sleep(1100);
        outtake.setBucketPos(outtake.bucketRegPos);
    }
    public static void lineup(LinearOpMode opMode){
        double power = 0.85;
        while((huskCam.getBlocksPos().get(0)[0] < 145) ||(huskCam.getBlocksPos().get(0)[0] > 175)) {
            if (huskCam.getBlocksPos().get(0)[0] < 145) {
                Robot.drive(power, -power, -power, power);
            } else if (huskCam.getBlocksPos().get(0)[0] > 175) {
                Robot.drive(-power, power, power, -power);
            } else {
                Robot.drive(0, 0, 0, 0);
            }
        }
        opMode.sleep(250);
        ad.goToPointConstantHeading(8.5, ad.getY() + 5.5);
        outtake.closeClaw();
        opMode.sleep(200);
        ad.goToPointConstantHeading(24,72);
        ad.goToHeading(180);
        ad.goToPointConstantHeading(40,72);
        clip(opMode);

    }


}
