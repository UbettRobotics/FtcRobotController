package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Robot.*;
@TeleOp(name="Motor Test")
public class Motortester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, true);
        waitForStart();
        int direction = 1;
        boolean prevoptions = false;
        while (opModeIsActive()){
            c.update();
            if(c.options && !prevoptions){
                direction = (direction == 1)?-1:1;
                prevoptions = true;
            }else if(!c.options && prevoptions){
                prevoptions = false;
            }
            if(c.a1){
                lf.setPower(1 * direction);
            }else {
                lf.setPower(0);
            }
            if(c.b1){
                rf.setPower(1 * direction);
            }else {
                rf.setPower(0);
            }
            if(c.x1){
                lb.setPower(1 * direction);
            }else {
                lb.setPower(0);
            }
            if(c.y1){
                    rb.setPower(1 * direction);
            }else {
                rb.setPower(0);
            }
        }
    }
}
