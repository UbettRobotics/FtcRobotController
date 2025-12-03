package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "test Auto 1")
public class TestAuto1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, 0);
        ad2.setOutputInfo(true);
        ad2.setTimeLimit(4);

        initAll(this, 1, false);
        ad2.setOutputInfo(true);
        ad2.setTimeLimit(3.5);
        ad2.resetOdo(this, 48.919, -51.185, 123.855);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Going...");
        telemetry.update();



        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        ad2.goToPointConstantHeading(0,0);
        sleep(5000);
        ad2.goToHeading(180);


    }

}
