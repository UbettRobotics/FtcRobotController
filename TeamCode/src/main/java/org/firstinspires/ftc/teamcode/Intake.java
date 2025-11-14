package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {
    DcMotorEx intakeMotor;
    LinearOpMode opMode;
    double intakeSpeed = 0.67;

    public Intake(LinearOpMode opMode){
        this.opMode = opMode;
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //0 -- none
    //1 -- purple
    //2 -- green
    //0 -- stop
    //1 -- in
    //2 -- out
    public void intakeArt(int direction){
        if(direction == 0){
            intakeMotor.setPower(0);
        }else if(direction == 1) {
            intakeMotor.setPower(intakeSpeed);
        }else if(direction == 2) {
            intakeMotor.setPower(-0.75 * intakeSpeed);
        }
    }

}
