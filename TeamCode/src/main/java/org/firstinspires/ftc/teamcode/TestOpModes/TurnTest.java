package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousDrive2;

@Autonomous(name = "Turn Test", group = "Test Modes")
public class TurnTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDrive2 ad2 = new AutonomousDrive2(this, true);


        double t = 0;
        double x = 0;

        double start = 0;
        double diff = 1;
        ad2.setOutputInfo(true);
        ad2.setTimeLimit(4);
        telemetry.addData("Pos X: ", ad2.getX());
        telemetry.addData("Pos Y: ", ad2.getY());
        telemetry.addData("Pos Heading: ", ad2.getHeading());
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            for(int i = 0; i < 8; i++){
                ad2.goToHeading(180 + i * 90);
                sleep(500);


            }

        }

    }
}
