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
        telemetry.addData("startx", ad.getX());
        telemetry.addData("starty", ad.getY());
        telemetry.update();

        waitForStart();
        telemetry.addData("startx", ad.getX());
        telemetry.addData("starty", ad.getY());
        telemetry.update();
        sleep(200);
        ad.goToHeading(0);
        ad.goToPoint(48,24,90,this);
        sleep(1000);
        ad.goToPoint(8.8,36,0,this);
        sleep(3000);
    }
}
