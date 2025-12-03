package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Robot.ad2;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto Red Far 1", preselectTeleOp = "Red Telop")
public class AutoRedFar1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, 1, false);
        ad2.setOutputInfo(true);
        ad2.setTimeLimit(4);
        ad2.resetOdo(this, -63.5, -24, -180);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        //Go launch Ball 1
        ad2.goToPointLinear(24, -24, 315);
        ad2.goToHeading(305);
        autoLaunch(this,outtake.launchMotor, outtake.loaderServo, outtake.loadingPos, outtake.launchPos);
        outtake.resetLoader();
        outtake.stopLaunch();

        //Go get Ball 2
        intake.intakeArt(1);
        ad2.goToPointLinear(-36, -18, 90);
        ad2.goToPointConstantHeading(-36,-40);
        intake.intakeArt(0);

        //Go launch Ball 2
        ad2.goToPointLinear(24, -24, 315);
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


