package org.firstinspires.ftc.teamcode.telops;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Red Telop")
public class Telop1Red extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, 1, true);
        Pose2D pose2D = ad2.getPos();


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
            telemetry.addData("X Pos: ", ad2.getX());
            telemetry.addData("Y Pos: ", ad2.getY());
            telemetry.addData("Dist: ", distance);
            telemetry.addData("launchSpeed: ", outtake.getLaunchSpeed()[0]);
            telemetry.update();

        }
    }

}
