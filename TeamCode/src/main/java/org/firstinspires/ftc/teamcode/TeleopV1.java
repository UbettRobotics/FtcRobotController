package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Robot.*;


@TeleOp(name = "teleop")
public class TeleopV1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);
        initIMU(this);

        waitForStart();

        while(opModeIsActive()) {
            boolean LBumper1 = gamepad1.left_bumper;
            boolean RBumper1 = gamepad1.right_bumper;

            double LStickY = gamepad1.left_stick_y;
            double LStickX  =  -gamepad1.left_stick_x;
            double RStickY = -gamepad1.right_stick_y;
            double RStickX = -gamepad1.right_stick_x;

            double LTrigger1 = -gamepad1.left_trigger;
            double RTrigger1 = -gamepad1.right_trigger;

            boolean a1 = gamepad1.a;
            boolean b1 = gamepad1.b;
            boolean x1 = gamepad1.x;
            boolean y1 = gamepad1.y;

            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            boolean x2 = gamepad2.x;
            boolean y2 = gamepad2.y;

            double LTrigger2 = gamepad2.left_trigger;
            double RTrigger2 = gamepad2.right_trigger;
            boolean LBumper2 = gamepad2.left_bumper;
            boolean RBumper2 = gamepad2.right_bumper;

            double RStickY2 = -gamepad2.right_stick_y;
            double RStickX2 = gamepad2.right_stick_x;
            double LStickY2 = -gamepad2.left_stick_y;
            double LStickX2 = gamepad2.left_stick_x;

            boolean dpadUp1 = gamepad1.dpad_up;
            boolean dpadRight1 = gamepad1.dpad_right;
            boolean dpadLeft1 = gamepad1.dpad_left;
            boolean dpadDown1 = gamepad1.dpad_down;

            boolean dpadUP2 = gamepad2.dpad_up;
            boolean dpadDOWN2 =gamepad2.dpad_down;
            boolean dpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;


            //diagonal driving
            if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
                if (Math.abs(LStickX) < .05 && Math.abs(RStickX) < .05) {
                    SetPower(LStickY, LStickY, LStickY, LStickY);
                }
                else if (Math.abs(LStickY) < .05 && Math.abs(RStickX) < .05) {
                    SetPower(LStickX, -LStickX, -LStickX, LStickX);
                }
                else {
                    double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                    double rightX = -gamepad1.right_stick_x;

                    double v1 = r * Math.cos(robotAngle) + rightX; //lf
                    double v2 = r * Math.sin(robotAngle) - rightX; //rf
                    double v3 = r * Math.sin(robotAngle) + rightX; //lb
                    double v4 = r * Math.cos(robotAngle) - rightX; //rb

                    SetPower(v1, v2, v3, v4);
                }
            }
            /*
            Trigger Turning
            else if (Math.abs(LTrigger1) > 0) {
                SetPower(-.5 * LTrigger1, .5 * LTrigger1, -.5 * LTrigger1, .5 * LTrigger1);
            }
            else if (Math.abs(RTrigger1) > 0) {
                SetPower(.5 * RTrigger1, -.5 * RTrigger1, .5 * RTrigger1, -.5 * RTrigger1);
            }
            */
            else if (LBumper1) {
                SetPower(-.5 * 1, .5 * 1, -.5 * 1, .5 * 1);
            }
            else if (RBumper1) {
                SetPower(.5 * 1, -.5 * 1, .5 * 1, -.5 * 1);
            }
            else {
                SetPower(0,0,0,0);
            }


/*
            //driving
            if (Math.abs(LStickY) > 0) {
                SetPower(LStickY, LStickY, LStickY, LStickY);
            }
            else if(Math.abs(RStickX) > 0) {
                SetPower(RStickX, -RStickX, RStickX, -RStickX);
            }
            else if (Math.abs(LTrigger1) > 0) {
                SetPower(-LTrigger1, LTrigger1, LTrigger1, -LTrigger1);
            }
            else if (Math.abs(RTrigger1) > 0) {
                SetPower(RTrigger1, -RTrigger1, -RTrigger1, RTrigger1);
            }
            else if (leftBump1) {
                SetPower(.3, -.3, .3, -.3);
            }
            else if (rightBump1) {
                SetPower(-.3, .3, -.3, .3);
            }
            else {
                SetPower(0, 0, 0, 0);
            }
*/

            //d pad fine tuned driving
            if(dpadUp1){
                SetPower(-.3, -.3, -.3, -.3); //0.3
            }
            else if(dpadRight1){
                SetPower(-.5, .5, .5, -.5); //0.5
            }
            else if(dpadLeft1){
                SetPower(.5, -.5, -.5, .5);
            }
            else if(dpadDown1){
                SetPower(.3, .3, .3, .3);
            }



            //controller 2////////////////////////////////////////////////////////////////////////////////
            double launchPower = 0;

            //launcher motors
            if(a2){
                //launcher1.setPower(0.80);
                launchPower = 0.75; //0.75
                blocker.setPosition(BLOCKER_OPEN);
            } else if (LTrigger2 > 0) {
                launchPower = 0.85; //0.9
            } else if (LBumper2) {
                launchPower = 1; //full
            } else if (dpadRight2) {
                launchPower = -0.1; //backwards
            } else if (RStickX2 > 0.2 || RStickY2 > 0.2)  {
                launchPower = 0.64;
            } else {
                launchPower = 0;
            }
            launcher2.setPower(launchPower);

            if(a1) {
                blocker.setPosition(BLOCKER_OPEN);
            } else if (!a2) {
                blocker.setPosition(BLOCKER_CLOSED);
            }

            //feeder servo
            if(dpadUP2){
                feeder.setPower(1);
                feederMotor.setPower(.5);
            } else if(dpadDOWN2){
                feeder.setPower(-1);
                feederMotor.setPower(-.5);
            } else {
                feeder.setPower(0);
                feederMotor.setPower(0);
            }

            //Launcher conveyor belt
            if(RTrigger2 > 0) {
                launcherbelt.setPower(1);
                feeder.setPower(1);
            } else if (RBumper2) {
                launcherbelt.setPower(-0.20);
                blocker.setPosition(0.65);
            } else {
                launcherbelt.setPower(0);
            }


            //open wobble claw
            if(x2){
                //open
                wobbleClaw.setPosition(0.7);
            } else {
                //close
                wobbleClaw.setPosition(0);
            }

            //wobble arm
            if(y2){
                ///lift up
                wobbleArmMotor.setTargetPosition(0);
                wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArmMotor.setPower(.375);
            }
            if(b2) {
                //go down
                wobbleArmMotor.setTargetPosition(351);
                wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArmMotor.setPower(.5);
            }


            //telementry////////////////////////////////////////////////////////////////////////////////////////////
            telemetry.addData("angle: ", getAngle());
            telemetry.addData("Belt encoder value: ", launcherbelt.getCurrentPosition());
            telemetry.addData("arm encoder: ", wobbleArmMotor.getCurrentPosition());
            telemetry.addData("wheel encoder: ", leftfront.getCurrentPosition());
            telemetry.addData("launcher Power: ", launcher2.getPower());
            telemetry.update();
        }
    }
}


