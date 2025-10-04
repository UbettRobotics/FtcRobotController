package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.driveFC;
import static org.firstinspires.ftc.teamcode.Robot.getRelativeTargetAngle;
import static org.firstinspires.ftc.teamcode.Robot.lineup;
import static org.firstinspires.ftc.teamcode.Robot.odo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Field Cent Demo")
public class FeildCentricDive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.intAll(this,0);


        waitForStart();


        while(opModeIsActive()) {
            odo.update();



            telemetry.addData("Heading: ", odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("X: ", odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y: ", odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("DistL: ", Robot.distL.getDistance(DistanceUnit.INCH));
            telemetry.addData("DistR: ", Robot.distR.getDistance(DistanceUnit.INCH));
            telemetry.addData("diff: ", Robot.distL.getDistance(DistanceUnit.INCH) - Robot.distR.getDistance(DistanceUnit.INCH));
            telemetry.update();

            if(gamepad1.left_bumper) {
                lineup();
            }else{
               driveFC(this);
            }


        }
    }
 }
