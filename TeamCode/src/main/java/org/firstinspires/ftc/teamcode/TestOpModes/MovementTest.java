package org.firstinspires.ftc.teamcode.TestOpModes;
import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousDrive2;

@Autonomous(name = "Test Moves", group = "Test Modes")
public class MovementTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDrive2 ad2 = new AutonomousDrive2(this,false);

        initAccessories(this);

        double t = 0;

        ad2.setOutputInfo(true);
        ad2.setTimeLimit(4);


        telemetry.addData("Pos X: ", ad2.getX());
        telemetry.addData("Pos Y: ", ad2.getY());
        telemetry.addData("Pos Heading: ", ad2.getHeading());
        telemetry.update();
        waitForStart();


        while(opModeIsActive()) {
            ad2.goToPointConstantHeading(48, 0);
            ad2.goToHeading(90);
            ad2.goToPointConstantHeading(24, 12);
            ad2.goToHeading(180);
            ad2.goToPointConstantHeading(0, 0);
            ad2.goToHeading(180);
            sleep(100);
            ad2.setPID(0.045,0.001,0.005-t,0);
            telemetry.clearAll();
            telemetry.addData("P: ", 0.006+t);
            telemetry.update();
            t += 0.0001;
            sleep(500);
        }

    }
}
