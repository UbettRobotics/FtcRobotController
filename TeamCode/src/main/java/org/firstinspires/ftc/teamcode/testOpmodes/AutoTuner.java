package org.firstinspires.ftc.teamcode.testOpmodes;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Tuner 2")
public class AutoTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, 0);
        ad2.setOutputInfo(true);
        ad2.setTimeLimit(4);
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Going...");
        telemetry.update();

        while(opModeIsActive()) {


            for (int i = 0; i <= 4; i++) {
                ad2.goToHeading(i * 90 + 180);
                sleep(250);
            }
            ad2.goToPointLinear(24,-24,90);
            sleep(250);
            ad2.goToPointLinear(48, 0, 180);
            sleep(250);
            ad2.goToPointLinear(24, 24, 270);
            sleep(250);
            ad2.goToPointLinear(0, 0, 180);
            sleep(1000);
        }


    }
}
