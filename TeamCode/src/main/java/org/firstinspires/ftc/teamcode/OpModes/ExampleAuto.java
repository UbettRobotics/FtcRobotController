package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Robot.*;

//@Autonomous (name="ex. auto")
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
        ad.goToPoint(36, 24,180);
        sleep(5000);
        ad.goToPoint(8.5,36,0);



        sleep(3000);
    }
}
