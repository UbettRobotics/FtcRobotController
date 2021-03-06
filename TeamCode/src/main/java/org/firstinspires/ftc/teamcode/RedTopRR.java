package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Robot.*;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Auto Red Top RR", preselectTeleOp = "teleop")
public class RedTopRR extends LinearOpMode {

    OpenCvCamera webcam;
    final int ARM_DOWN = 388;
    final int ARM_UP = 0;
    final double ARM_POWER = 0.4;
    private Trajectory moveToZoneAgain = null;

    @Override
    public void runOpMode() throws InterruptedException {
        ////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        //init drive train
        MainMecanumDrive drive = new MainMecanumDrive(hardwareMap);
        Robot.initAccessories(this);
        Pose2d startPose = new Pose2d(57, 20, Math.toRadians(0)); //init starting position
        drive.setPoseEstimate(startPose); //important
        //init servos
        blocker.setPosition(BLOCKER_OPEN);
        wobbleClaw.setPosition(0);

        //////Start Camera Streaming//////
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        RingVisionPipeline pipeline = new RingVisionPipeline(telemetry);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

////////Program start////////////////////////////////////////////////////////////////////////
        waitForStart();

        telemetry.addData("location: ", pipeline.getLocation());
        telemetry.update();



        //store the value of the ring stack
        int stackPos;
        switch (pipeline.getLocation()) {
            case C_FULL_STACK:
                stackPos = 3;
                break;
            case B_HALF_STACK:
                stackPos = 2;
                break;
            case A_NO_STACK:
                stackPos = 1;
                break;
            default: stackPos = 1;
        }



        //move to first pos
        Trajectory initForward = drive.trajectoryBuilder(startPose,true)
                .splineTo(new Vector2d(-4,20), Math.toRadians(180))
                .build();
        drive.followTrajectory(initForward);

        //Ramp up launcher
        //
        // V
        launcher2.setPower(0.85);
        // ^
        //
        //move to first shot
        Trajectory shot1 = drive.trajectoryBuilder(initForward.end())
                .lineToLinearHeading(new Pose2d(-7.5, 43, Math.toRadians(0)))
                .build();
        drive.followTrajectory(shot1);

        //launch ring1
        sleep(500);
        launcherbelt.setTargetPosition(900);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(0.9);
        while(launcherbelt.isBusy()) {}
        //launch ring2
        launcherbelt.setTargetPosition(3000);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(0.9);
        while(launcherbelt.isBusy()) {}
        //launch ring3
        launcherbelt.setTargetPosition(5100);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(0.9);
        while(launcherbelt.isBusy()) {}
        launcher2.setPower(0);

        ///////////three different paths depending on drop zone of wobble goal////////////

        switch (stackPos) {
            case 3:
                //FULL Stack rings
                zoneC(drive, shot1);
                break;
            case 2:
                //HALF stack rings
                zoneB(drive, shot1);
                break;
            case 1:
                //NO rings
                zoneA(drive, shot1);
                break;
            default:
                moveToZoneAgain = drive.trajectoryBuilder(shot1.end(),true)
                        .splineTo(new Vector2d(-5,40), Math.toRadians(-90))
                        .build();
                drive.followTrajectory(moveToZoneAgain);
        }

        feeder.setPower(0);
        sleep(1000);
        ejectWobbleGoal();


        Trajectory moveToLine = drive.trajectoryBuilder(moveToZoneAgain.end(),true)
                .lineToLinearHeading(new Pose2d(-15,43, Math.toRadians(0)))
                .build();
        drive.followTrajectory(moveToLine);

        launcher2.setPower(0);


        if (isStopRequested()) return;
    }


    private void zoneA(MainMecanumDrive drive, Trajectory shot1){
        Trajectory moveToZone;
        Trajectory moveToWobble2;
        Trajectory approachWobble2;
        moveToZone = drive.trajectoryBuilder(shot1.end(),true)
                .lineToLinearHeading(new Pose2d(-10, 54, Math.toRadians(115)))
                .build();
        drive.followTrajectory(moveToZone);
        ejectWobbleGoal();

        intake();

        moveToWobble2 = drive.trajectoryBuilder(moveToZone.end(), true)
                .lineToLinearHeading(new Pose2d(10, 49, Math.toRadians(0)))
                .build();
        drive.followTrajectory(moveToWobble2);

        moveWobbleArmDown();

        approachWobble2 = drive.trajectoryBuilder(moveToWobble2.end(),true)
                .forward(13)
                .build();
        drive.followTrajectory(approachWobble2);

        pickUpWobble();

        moveToZoneAgain = drive.trajectoryBuilder(moveToWobble2.end())
                .lineToLinearHeading(new Pose2d(-5, 51, Math.toRadians(143)))
                .build();
        drive.followTrajectory(moveToZoneAgain);
    }

    private void zoneB(MainMecanumDrive drive, Trajectory shot1){
        Trajectory moveToZone;
        Trajectory moveToWobble2;
        Trajectory approachWobble2;
        Trajectory moveToRing;


        moveToZone = drive.trajectoryBuilder(shot1.end(),true)
                .splineTo(new Vector2d(-30.5,35), Math.toRadians(-45))
                .build();
        drive.followTrajectory(moveToZone);
        ejectWobbleGoal();

        intake();
        blocker.setPosition(BLOCKER_CLOSED);

        moveToRing = drive.trajectoryBuilder(moveToZone.end())
                .lineToLinearHeading(new Pose2d(11,40.5, Math.toRadians(0)))
                .build();
        drive.followTrajectory(moveToRing);

        sleep(500);
        moveToWobble2 = drive.trajectoryBuilder(moveToRing.end(),true)
                .lineToLinearHeading(new Pose2d(14.5,48.8, Math.toRadians(0)))
                .build();
        drive.followTrajectory(moveToWobble2);

        moveWobbleArmDown();

        approachWobble2 = drive.trajectoryBuilder(moveToWobble2.end(),true)
                .forward(11.8)
                .build();
        drive.followTrajectory(approachWobble2);
        pickUpWobble();

        //start Ring2
        launcher2.setPower(.85);
        Trajectory secondShot = drive.trajectoryBuilder(approachWobble2.end())
                .lineToLinearHeading(new Pose2d(-5.75,43, Math.toRadians(0)))
                .build();
        drive.followTrajectory(secondShot);

        //Shoot
        launcherbelt.setPower(0);
        blocker.setPosition(BLOCKER_OPEN);
        sleep(200);
        launcherbelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherbelt.setTargetPosition(1500);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(0.85);
        while(launcherbelt.isBusy());
        launcher2.setPower(0);

        moveToZoneAgain = drive.trajectoryBuilder(secondShot.end(),true)
                .splineTo(new Vector2d(-22,32), Math.toRadians(-45))
                .build();
        drive.followTrajectory(moveToZoneAgain);
    }

    private void zoneC(MainMecanumDrive drive, Trajectory shot1){
        Trajectory moveToZone;
        Trajectory moveToWobble2;
        Trajectory approachWobble2;
        Trajectory backUp;
        Trajectory moveToRing;

        moveToZone = drive.trajectoryBuilder(shot1.end(),true)
                .lineToLinearHeading(new Pose2d(-53, 57, Math.toRadians(145)))
                .build();
        drive.followTrajectory(moveToZone);
        ejectWobbleGoal();

        backUp = drive.trajectoryBuilder(moveToZone.end())
                .lineToLinearHeading(new Pose2d(-15, 19, Math.toRadians(45))) //-30, 40
                .build();

        moveToRing = drive.trajectoryBuilder(backUp.end())
                .lineToLinearHeading(new Pose2d(13,36, Math.toRadians(30)))
                .build();
        drive.followTrajectory(backUp);

        blocker.setPosition(BLOCKER_CLOSED);
        intake();


        drive.followTrajectory(moveToRing);

        moveToWobble2 = drive.trajectoryBuilder(moveToRing.end())
                .lineToLinearHeading(new Pose2d(15.5, 34, Math.toRadians(27)))
                .build();
        drive.followTrajectory(moveToWobble2);

        //move arm down
        moveWobbleArmDown();



        approachWobble2 = drive.trajectoryBuilder(moveToWobble2.end(),true)
                .forward(16.5)
                .build();
        drive.followTrajectory(approachWobble2);

        launcher2.setPower(.85);

        pickUpWobble();


        //start Ring2

        Trajectory secondShot = drive.trajectoryBuilder(approachWobble2.end())
                .lineToLinearHeading(new Pose2d(-5,40, Math.toRadians(0)))
                .build();
        drive.followTrajectory(secondShot);


        //Shoot
        blocker.setPosition(BLOCKER_OPEN);
        launcherbelt.setPower(0);
        sleep(200);
        launcherbelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherbelt.setTargetPosition(2500);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(0.85);
        while(launcherbelt.isBusy());
        launcher2.setPower(0);

        moveToZoneAgain = drive.trajectoryBuilder(secondShot.end(),true)
                .lineToLinearHeading(new Pose2d(-49, 50, Math.toRadians(115)))
                .build();
        drive.followTrajectory(moveToZoneAgain);
    }





    private void pickUpWobble(){
        wobbleClaw.setPosition(0); //closed
        sleep(1000);
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() > ARM_UP + 30){}

    }

    private void ejectWobbleGoal() {
        wobbleArmMotor.setTargetPosition(ARM_DOWN);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() < ARM_DOWN - 30){}
        wobbleClaw.setPosition(1);
        sleep(500);
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() > ARM_UP + 30){}
        wobbleClaw.setPosition(0);
    }

    private void moveWobbleArmDown(){
        //move arm down
        wobbleClaw.setPosition(1);
        wobbleArmMotor.setTargetPosition(ARM_DOWN);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() < ARM_DOWN - 10){}
    }
    private void intake() {
        //intake
        feeder.setPower(1);
        launcherbelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherbelt.setTargetPosition(16000);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(1);
    }
}
