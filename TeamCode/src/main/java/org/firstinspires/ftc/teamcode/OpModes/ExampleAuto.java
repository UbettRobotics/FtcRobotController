package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.teamcode.AutonomousDrive2;

@Autonomous (name="ex. auto")
public class ExampleAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, true);
        outtake.stopVSlide();
        outtake.setBucketPos(outtake.bucketRegPos);
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();
        telemetry.addData("startx", ad.getX());
        telemetry.addData("starty", ad.getY());
        telemetry.update();

        waitForStart();
        ad.goToHeading(0);
        ad.goToPointConstantHeading(48,36);
        ad.goToHeading(0);
        ad.goToPointConstantHeading(12,36);
        ad.goToHeading(0);
        ad.goToPointConstantHeading(12,72);
        ad.goToHeading(0);
        ad.goToPointConstantHeading(12,36);
        ad.goToHeading(0);
        ad.goToPointConstantHeading(8.5,36);
        ad.goToHeading(0);


        sleep(3000);
    }
}
