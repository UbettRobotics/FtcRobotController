package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TelopBlue1")
public class Telop1Blue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.intAll(this,0);

        waitForStart();

        while (opModeIsActive()){
            Robot.driveFC(this);
            if(gamepad1.a){

            }
        }
    }
}
