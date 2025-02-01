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
        for(int i = 0; i <= 360; i+=90){
            ad.goToHeading(i);
            this.sleep(500);
        }



        sleep(3000);
    }
}
