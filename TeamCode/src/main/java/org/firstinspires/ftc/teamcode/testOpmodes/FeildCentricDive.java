package org.firstinspires.ftc.teamcode.testOpmodes;

import static org.firstinspires.ftc.teamcode.Robot.*;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Field Cent Demo")
public class FeildCentricDive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.initAll(this,0);


        waitForStart();


        while(opModeIsActive()) {
            odo.update();



            telemetry.addData("Heading: ", odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("X: ", odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y: ", odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading angle: ", da.getRelativeTargetAngle(48,48));

            telemetry.update();

            if(gamepad1.left_bumper) {
                lineup();
            }else{
               driveFC(this);
            }


        }
    }
 }
