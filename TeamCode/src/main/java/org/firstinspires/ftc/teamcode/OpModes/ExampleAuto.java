package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Robot.*;

@Autonomous (name="ex. auto")
public class ExampleAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, true);
        outtake.stopVSlide();
        outtake.setBucketPos(outtake.bucketRegPos);
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();

        waitForStart();
        telemetry.addData("startx", ad.getX());
        telemetry.addData("starty", ad.getY());
        telemetry.update();
        sleep(200);
        ad.goToHeading(0);
        ad.goToPointConstantHeading(60,24);
        sleep(1000);
        ad.goToHeading(270);
        ad.goToPointConstantHeading(8.6,36);
        ad.goToHeading(0);


        sleep(3000);
    }
}
