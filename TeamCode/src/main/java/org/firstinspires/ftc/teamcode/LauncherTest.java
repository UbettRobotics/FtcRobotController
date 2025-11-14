package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Launch Test")
public class LauncherTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, 1);


        double launchSpeed = 0;
        boolean lastBumper1 = false;
        boolean lastBumper2 = false;

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.right_bumper && !lastBumper1){
                launchSpeed += 1000;
                lastBumper1 = true;
            }else if(!gamepad1.right_bumper && lastBumper1){
                lastBumper1 = false;
            }else if(gamepad1.left_bumper&& !lastBumper2 && launchSpeed > 0){
                launchSpeed -= 1000;
                lastBumper2 = true;
            }else if(!gamepad1.left_bumper && lastBumper2){
                lastBumper2 = false;
            }else if(Math.abs(gamepad1.left_stick_y) > 0.01){
                launchSpeed += gamepad1.left_stick_y * 100;
            }

            if(gamepad1.a){
                outtake.loaderServo.setPosition(outtake.launchPos);
            }else{
                outtake.loaderServo.setPosition(outtake.loadingPos);
            }

            outtake.setLauncherRpm(launchSpeed);
            telemetry.addData("Actual Rpm: ", outtake.getSpeedofLauncher());
            telemetry.addData("Input Velo Rpm: ", (launchSpeed/60*28));

            telemetry.addData("Launch Speed: ", launchSpeed);
            telemetry.update();

        }

    }
}
