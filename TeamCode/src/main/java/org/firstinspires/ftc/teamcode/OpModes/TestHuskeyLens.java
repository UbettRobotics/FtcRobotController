package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Robot.*;


@TeleOp(name = "Husky Test")
public class TestHuskeyLens extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        initAll(this,true);
        waitForStart();
        while(opModeIsActive()){
            huskCam.outPutInfo(this);
        }
    }
}
