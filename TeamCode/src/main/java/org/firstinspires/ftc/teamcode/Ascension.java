package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Ascension {

    //Objects
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public int upPos = 1000;
    public int downPos = 0;

    public boolean up = false;


    //Constructor
    public Ascension(OpMode opMode){
        leftMotor = opMode.hardwareMap.get(DcMotor.class, "leftAscension");
        rightMotor = opMode.hardwareMap.get(DcMotor.class, "rightAscension");
    }


    //Vertical Slide to Position
    public void slidesToPos(int pos, double power){
        leftMotor.setTargetPosition(pos);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(power);

        rightMotor.setTargetPosition(pos);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(power);
    }

    public void slidesToPow(double power){
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setPower(power);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setPower(power);
    }

    public void up(){
        slidesToPos(upPos, 1.0);
        up = true;
    }

    public void down(){
        slidesToPos(downPos, 1.0);
        up = false;

    }



}

