package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoRedClose2")
public class AutoRedClose2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, 1, false);
        ad2.setOutputInfo(true);
        ad2.setTimeLimit(3.5);
        ad2.resetOdo(this, 48.919, -51.185, 123.855);

        ad2.createPath(new double[]{36,-36, 315},270);
        ad2.getPath(0).addSpline(12,-18,90,0,15);
        ad2.getPath(0).addSpline(12,-40,90,90);

        outtake.resetLoader();
        outtake.stopLaunch();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        //Go launch Ball 1
        ad2.goToPointLinear(36, -30, 315);
        autoLaunch(this,outtake.launchMotor, outtake.loaderServo, outtake.loadingPos, outtake.launchPos);
        outtake.resetLoader();
        outtake.stopLaunch();

        // Ball 2
        ad2.goToHeading(90);
        intake.intakeArt(1);
        ad2.runPathConstantHeading(0,0.025);
        ad2.goToPointLinear(36, -30, 315);
        intake.intakeArt(0);
        autoLaunch(this,outtake.launchMotor, outtake.loaderServo, outtake.loadingPos, outtake.launchPos);
        sleep(500);
        outtake.resetLoader();

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


