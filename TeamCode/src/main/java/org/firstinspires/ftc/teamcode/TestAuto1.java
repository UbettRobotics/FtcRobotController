package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;

@Autonomous(name = "test Auto 1")
public class TestAuto1 extends LinearOpMode {

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

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();



        ad2.goToPointConstantHeading(48, -48);
        telemetry.addLine("Move 1 done");
        telemetry.update();
        sleep(500);
        ArrayList<Pose2D> points = new ArrayList<>();
        ArrayList<Double> weights = new ArrayList<>();
        points.add(new Pose2D(DistanceUnit.INCH,48,-48, AngleUnit.DEGREES, 180));
        points.add(new Pose2D(DistanceUnit.INCH,48,0, AngleUnit.DEGREES, 180));
        points.add(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES, 180));
        points.add(new Pose2D(DistanceUnit.INCH,0,-48, AngleUnit.DEGREES, 180));

        weights.add(1.0);
        weights.add(1.0);
        weights.add(1.0);
        weights.add(1.0);

        //ad2.createPath(points, weights);

        //ad2.runPath(0);

        ad2.goToHeading(0);
        sleep(500);
        ad2.goToPointLinear(48, 0,90);
        telemetry.addLine("Move 2 done");
        telemetry.update();
        sleep(500);
        ad2.goToHeading(90);
        ad2.goToPointLinear(-48, 0,270);
        telemetry.addLine("Move 3 done");
        telemetry.update();
        sleep(500);
        ad2.goToHeading(270);
        ad2.goToPointLinear(0, 0,180);
        telemetry.addLine("Move 4 done");
        telemetry.update();
        sleep(500);
        ad2.goToHeading(180);







    }

}
