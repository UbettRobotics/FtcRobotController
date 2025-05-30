package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.AutonomousDrive2;


@Autonomous(name = "Pid Tuner", group = "Test Modes")
public class PIDTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDrive2 ad = new AutonomousDrive2(this,true);
        ad.setOutputInfo(true);
        ad.setTimeLimit(4);
        telemetry.addData("Pos X: ", ad.getX());
        telemetry.addData("Pos Y: ", ad.getY());
        telemetry.addData("Pos Heading: ", ad.getHeading());
        telemetry.update();
        double kP = 0.1;
        double kI = 0;
        double kD = 0;
        waitForStart();
        for (int h = 0; h < 5; h += 1) {
            for (int j = 0; j < 5; j += 1) {
                for (int k = 0; k <= 5; k += 1) {
                    ad.setPID(kP + k/100.0, kI+ h/1000.0, kD + j/10000.0, 1);
                    telemetry.addData("P: ", kP + k/100.0);
                    telemetry.addData("I: ", kI+ h/1000.0);
                    telemetry.addData("D: ", kD + j/10000.0);
                    telemetry.update();
                    sleep(1500);
                    ad.goToPointConstantHeading(24,0);
                    sleep(500);
                    ad.goToHeadingOrg(180);
                    sleep(500);
                    ad.goToPointConstantHeading(0,0);
                    sleep(500);
                    ad.goToHeadingOrg(180);
                    sleep(500);

                }
            }
        }
    }
}
