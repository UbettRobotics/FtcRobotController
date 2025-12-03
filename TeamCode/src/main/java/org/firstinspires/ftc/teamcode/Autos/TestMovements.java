package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Robot.ad2;
import static org.firstinspires.ftc.teamcode.Robot.drive;
import static org.firstinspires.ftc.teamcode.Robot.initAll;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test Movements")
public class TestMovements extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException{
        initAll(this, 0);
        ad2.outputInfo();
        ad2.setTimeLimit(2);

        double t = 0;
        ad2.createPath(new double[]{0,0, 180}, 0);
        ad2.getPath(0).addSpline(24,-24,180,90, 20);
        ad2.getPath(0).addSpline(48,0,180,270,20);
        ad2.getPath(0).addSpline(0,0,180,0,10);
        ad2.getPath(0).addSpline(24,-24,0,90, 20);
        ad2.getPath(0).addSpline(48,0,270,270,20);
        ad2.getPath(0).addSpline(0,0,180,0);

                ad2.createPath(new double[]{0,0, 180},90);
                ad2.getPath(1).addSpline(12,-18,270,0,15);
                ad2.getPath(1).addSpline(12,-38,90,90,20);
                ad2.getPath(1).addSpline(36,-36,310,315, 10);




        waitForStart();


        telemetry.addLine("Try 1 Done:");
        telemetry.update();
        for(int i = 1; i <= 4; i++){
                ad2.goToHeading(180 + 90*i);
        }
        ad2.runPathConstantHeading(0,0.025);
        for(int i = 1; i <= 4; i++){
                ad2.goToHeading(180 + 90*i);
        }
        ad2.runPathConstantHeading(1,0.025);

        sleep(2600);

        ad2.goToPointLinear(0,0,180);

        telemetry.addLine("All Done!!");
        telemetry.update();
    }

}
