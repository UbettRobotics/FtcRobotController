package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.ad2;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto Blue Close 1", preselectTeleOp = "Blue Telop")
public class AutoBlueClose1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, 0, false);
        ad2.setOutputInfo(true);
        ad2.setTimeLimit(3.5);
        ad2.resetOdo(this, 50, 53, -135);
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        //Go launch Ball 1
        ad2.goToPointLinear(36, 24, 50);
        autoLaunch(this,outtake.launchMotor, outtake.loaderServo, outtake.loadingPos, outtake.launchPos);
        outtake.resetLoader();
        outtake.stopLaunch();

        //Go get Ball 2
        intake.intakeArt(1);
        ad2.goToPointLinear(11, 18, 270);
        ad2.goToPointConstantHeading(11,40);
        intake.intakeArt(0);

        //Go launch Ball 2
        ad2.goToPointLinear(36, 24, 50);
        autoLaunch(this,outtake.launchMotor, outtake.loaderServo, outtake.loadingPos, outtake.launchPos);
        outtake.resetLoader();
        outtake.stopLaunch();

        //Go get Ball 3
        intake.intakeArt(1);
        ad2.goToPointLinear(11, 24, 270);
        ad2.goToPointConstantHeading(11,50);
        intake.intakeArt(0);

        //Go launch Ball 3
        ad2.goToPointLinear(36, 24, 50);
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


