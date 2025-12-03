package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "Auto Red Close 1", preselectTeleOp = "Red Telop")
public class AutoRedClose1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, 1, false);
        ad2.setOutputInfo(true);
        ad2.setTimeLimit(3.5);
        ad2.resetOdo(this, 51, -51.5, 135);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        //Go launch Ball 1
        ad2.goToPointLinear(36, -24, 315);
        autoLaunch(this,outtake.launchMotor, outtake.loaderServo, outtake.loadingPos, outtake.launchPos);
        outtake.resetLoader();
        outtake.stopLaunch();

        //Go get Ball 2
        intake.intakeArt(1);
        ad2.goToPointLinear(12, -18, 90);
        ad2.goToPointConstantHeading(12,-40);
        intake.intakeArt(0);

        //Go launch Ball 2
        ad2.goToPointLinear(36, -24, 315);
        autoLaunch(this,outtake.launchMotor, outtake.loaderServo, outtake.loadingPos, outtake.launchPos);
        outtake.resetLoader();
        outtake.stopLaunch();

        //Go get Ball 3
        intake.intakeArt(1);
        ad2.goToPointLinear(12, -18, 90);
        ad2.goToPointConstantHeading(12,-50);
        intake.intakeArt(0);

        //Go launch Ball 3
        ad2.goToPointLinear(36, -24, 315);
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
        loadServo.setPosition(launchPose);
        sleep(800);
        launchMotor.setVelocity(0);
    }
}


