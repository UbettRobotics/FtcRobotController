package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.ad2;
import static org.firstinspires.ftc.teamcode.Robot.drive;
import static org.firstinspires.ftc.teamcode.Robot.initAll;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Test Movements")
public class TestMovements extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException{
        initAll(this, 0);
        ad2.outputInfo();
        ad2.setTimeLimit(2);

        double t = 0;
        ad2.createPath(new double[]{0,0, 180}, 0);
        ad2.getPath(0).addSpline(24,-24,180,90, 10);
        ad2.getPath(0).addSpline(48,0,180,270,4);
        ad2.getPath(0).addSpline(0,0,180,270);

        int size = ad2.getPath(0).path.size();
                telemetry.addData("Point X: ",ad2.getPath(0).path.get(size-2)[0]);
        telemetry.addData("Point Y: ",ad2.getPath(0).path.get(size-2)[1]);
        telemetry.update();




        waitForStart();


        telemetry.addLine("Try 1 Done:");
        telemetry.update();
        ad2.runPath(0,0.025);
        drive(0,0,0,0);

        telemetry.addLine("All Done!!");
        telemetry.update();
    }

}
