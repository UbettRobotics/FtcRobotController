package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "test telop")
public class TestTelop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, 1);

        waitForStart();

        while (opModeIsActive()) {
            driveFC(this);
            if (gamepad2.right_bumper) {
                intake.intakeArt(1);
            } else if (gamepad2.left_bumper) {
                intake.intakeArt(2);
            } else {
                intake.intakeArt(0);
            }

            if (gamepad2.a) {
                outtake.launchArt();
            } else if (gamepad2.dpad_up) {
                outtake.launchMotor.setPower(1);
            } else {
                outtake.resetLoader();
                outtake.stopLaunch();
            }
            if(gamepad2.right_trigger > 0.5){
                outtake.loaderServo.setPosition(outtake.launchPos);
            }else if(gamepad2.left_trigger > 0.5){
                outtake.loaderServo.setPosition(outtake.loadingPos);
            }

            double distance = Math.hypot((ad2.getX()-targetPosRed[0]), (ad2.getY()-targetPosRed[1]));
            telemetry.addData("Actual Rpm: ", outtake.getSpeedofLauncher());
            telemetry.addData("Launch Speed: ", outtake.launchSpeed);
            telemetry.addData("X Pos: ", ad2.getX());
            telemetry.addData("Y Pos: ", ad2.getY());
            telemetry.addData("Dist: ", distance);
            telemetry.addData("launchSpeed: ", outtake.getLaunchSpeed()[0]);
            telemetry.update();

        }
    }

    @Autonomous(name = "Auto Blue Far 1", preselectTeleOp = "Blue Telop")
    public static class AutoBlueFar1 extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            initAll(this, 0, false);
            ad2.setOutputInfo(true);
            ad2.setTimeLimit(4);
            ad2.resetOdo(this, -63.5, 24, 180);

            telemetry.addLine("Ready");
            telemetry.update();

            waitForStart();

            //Go launch Ball 1
            ad2.goToPointLinear(24, 24, 38);
            autoLaunch(this,outtake.launchMotor, outtake.loaderServo, outtake.loadingPos, outtake.launchPos);
            outtake.resetLoader();
            outtake.stopLaunch();

            //Go get Ball 2
            intake.intakeArt(1);
            ad2.goToPointLinear(-38, 18, 270);
            ad2.goToPointConstantHeading(-38,40);
            intake.intakeArt(0);

            //Go launch Ball 2
            ad2.goToPointLinear(24, 24, 38);
            autoLaunch(this,outtake.launchMotor, outtake.loaderServo, outtake.loadingPos, outtake.launchPos);
            outtake.resetLoader();
            outtake.stopLaunch();


        }

        public void autoLaunch(LinearOpMode opMode, DcMotorEx launchMotor, Servo loadServo, double loadPos, double launchPose) {
            launchMotor.setVelocity(2600);
            while (launchMotor.getVelocity() < 2500 && opModeIsActive()) {
                launchMotor.setVelocity(2600);
                loadServo.setPosition(loadPos);
                sleep(1);
                telemetry.addLine("Spinning Up");
                telemetry.update();
            }
            sleep(100);
            loadServo.setPosition(launchPose);
            sleep(1000);
            launchMotor.setVelocity(0);
        }
    }
}
