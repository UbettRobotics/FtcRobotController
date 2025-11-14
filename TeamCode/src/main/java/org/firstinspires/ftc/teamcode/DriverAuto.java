package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriverAuto {
    LinearOpMode opMode;

    double distBetween = 12.28;

    public  DistanceSensor distL, distR;


    public DriverAuto(LinearOpMode opMode){
        this.opMode = opMode;
        //distL = opMode.hardwareMap.get(DistanceSensor.class, "distsensL");
        //distR = opMode.hardwareMap.get(DistanceSensor.class, "distsensR");

    }

    public void lineup(){
        double dist1 = distL.getDistance(DistanceUnit.INCH);
        double dist2 = distR.getDistance(DistanceUnit.INCH);

        double dist3 = dist2 - dist1;


        double turn = ad2.turnPID(dist3);

        drive(turn,turn, -turn,-turn);
    }

    public double getRelativeTargetAngle(double x, double y){
        double out = 0;
        double xPos = ad2.getX();
        double yPos = ad2.getY();

        out = Math.atan2(y-yPos,x-xPos);
        return  (Math.toDegrees(out))% 360;
    }
}
