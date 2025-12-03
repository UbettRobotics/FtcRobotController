package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Test")
public class Motortest extends LinearOpMode {
    public static DcMotor[] motors = new DcMotor[4];

    @Override
    public void runOpMode() throws InterruptedException {
          DcMotorEx rf;
          DcMotorEx rb;
          DcMotorEx lb;
         DcMotorEx lf;

        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");



        motors[0] = rf;
        motors[1] = rb;
        motors[2] = lb;
        motors[3] = lf;


        //0 ports is the back motors config and both motors are going to both control and expansion hub
        //Encoders go to those ports


        for (int i = 0; i < 4; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);


        double speed = -1;
        int motor = 0;

        waitForStart();

        while (opModeIsActive()){
            switch (motor){
                case 0:
                    lf.setPower(speed);
                    break;
                case 1:
                    lb.setPower(speed);
                    break;
                case 2:
                    rb.setPower(speed);
                    break;
                case 3:
                    rf.setPower(speed);
                    break;
            }
            if(speed > 1){
                speed = -1;
                lf.setPower(0);
                lb.setPower(0);
                rf.setPower(0);
                rb.setPower(0);
                motor++;
            }else{
                speed += 0.1;
            }
            sleep(500);
        }

        telemetry.addData("Motor: ", motor);
        telemetry.addData("speed: ", speed);
        telemetry.update();

    }
}
