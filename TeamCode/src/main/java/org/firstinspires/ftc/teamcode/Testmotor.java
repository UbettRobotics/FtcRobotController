package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Robot.*;

@TeleOp(name = "Other")
public class Testmotor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Control c = new Control(this);
        initDrive(this);

        waitForStart();
        while(opModeIsActive()) {
            c.update();
           // slideout.setPower(c.RTrigger1);
            //slideout.setPower(-c.LTrigger1);
        }
    }
}