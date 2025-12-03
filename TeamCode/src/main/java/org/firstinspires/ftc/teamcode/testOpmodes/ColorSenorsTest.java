package org.firstinspires.ftc.teamcode.testOpmodes;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Color Sensors test")
public class ColorSenorsTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this,1);

        double startTime = getRuntime();
        while(getRuntime() - startTime < 2){
            int[] rgb = intake.getColorIntake();
            intake.normalizeColorVals();
            telemetry.addLine("Normalizing...");
            telemetry.addData("Red: ", rgb[0]);
            telemetry.addData("Green: ", rgb[1]);
            telemetry.addData("Blue: ", rgb[2]);
            telemetry.addLine("Normalized vals");
            telemetry.addData("Red: ", intake.normalizedRed);
            telemetry.addData("Green: ", intake.normalizedGreen);
            telemetry.addData("Blue: ", intake.normalizedBlue);
            telemetry.update();
        }
        telemetry.addLine("Normalzed!");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            int[] rgb = intake.getColorIntake();
            telemetry.addData("Red: ", rgb[0]);
            telemetry.addData("Green: ", rgb[1]);
            telemetry.addData("Blue: ", rgb[2]);
            telemetry.addLine("Normalized vals");
            telemetry.addData("Red: ", intake.normalizedRed);
            telemetry.addData("Green: ", intake.normalizedGreen);
            telemetry.addData("Blue: ", intake.normalizedBlue);
            telemetry.addData("What Color: ", intake.haveArtinIntake());
            telemetry.update();
        }
    }
}
