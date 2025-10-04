package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "test Auto 1")
public class TestAuto1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        intAll(this, 0);
        ad2.setOutputInfo(true);
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Going...");
        telemetry.update();


        ad2.goToPointConstantHeading(48, 0);
        sleep(1000);
        ad2.goToPointLinear(24, 24, 90);
        sleep(1000);
        ad2.goToPointLinear(0, 0, 0);
        sleep(1000);



    }
}
