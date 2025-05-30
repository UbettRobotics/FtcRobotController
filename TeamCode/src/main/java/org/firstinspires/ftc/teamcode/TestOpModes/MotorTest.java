package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.AutonomousDrive2;

@Autonomous(name = "motor tests" , group = "Test Modes")

public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Drive motor vars
         DcMotorEx lf;
         DcMotorEx lb;
         DcMotorEx rf;
         DcMotorEx rb;

         DcMotorControllerEx motorControllerEx;

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        motorControllerEx = (DcMotorControllerEx) lf.getController();

        double testTime = 1;



        waitForStart();
        double startTimer = time;


        for(int i = 0; i <4; i++){
            if(i == 0){
                rf.setPower(1);
                rb.setPower(0);
                lb.setPower(0);
                lf.setPower(0);
                startTimer = time;
                while(time - startTimer < testTime){
                    telemetry.addData("RF Speed: ", motorControllerEx.getMotorVelocity(rf.getPortNumber()));
                    telemetry.update();
                }
            }else if(i == 1){
                rf.setPower(0);
                rb.setPower(1);
                lb.setPower(0);
                lf.setPower(0);
                startTimer = time;
                while(time - startTimer < testTime){
                    telemetry.addData("RB Speed: ", motorControllerEx.getMotorVelocity(rb.getPortNumber()));
                    telemetry.update();
                }
            }
            else if(i == 2){
                rf.setPower(0);
                rb.setPower(0);
                lb.setPower(0);
                lf.setPower(1);
                startTimer = time;
                while(time - startTimer < testTime){
                    telemetry.addData("LF Speed: ", motorControllerEx.getMotorVelocity(lf.getPortNumber()));
                    telemetry.update();
                }
            }
            else if(i == 3){
                rf.setPower(0);
                rb.setPower(0);
                lb.setPower(1);
                lf.setPower(0);
                startTimer = time;
                while(time - startTimer < testTime){
                    telemetry.addData("LB Speed: ", motorControllerEx.getMotorVelocity(lb.getPortNumber()));
                    telemetry.update();
                }
            }
        }
            rf.setPower(0);
            rb.setPower(0);
            lb.setPower(0);
            lf.setPower(0);
    }


}
