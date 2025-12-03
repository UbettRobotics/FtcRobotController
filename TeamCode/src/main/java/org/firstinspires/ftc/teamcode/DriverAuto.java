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


    public void goPark(int side){
        if(side == 0){

        }
    }



    public void goToPointLinearNoStopDa(double targetX, double targetY, double targetHeading){
        odo.update();

        ad2.errorSumDX2 = 0;
        ad2.errorSumDY2 = 0;
        ad2.errorSumT = 0;


        double targetXDist =  (targetX- ad2.getX());
        double targetYDist = (ad2.getY() - targetY);
        double totalDist = Math.hypot(targetXDist, targetYDist);


        double startHeading = ad2.getHeading();
        double turn = -ad2.turnPID(startHeading);

        double startTime = opMode.time;
        double startTime2 = opMode.time;

        double startDist = totalDist;
        ad2.targetTime = opMode.time;


        while(!ad2.checkTime(startTime,opMode.time) &&
                (Math.abs(targetXDist) > ad2.POS_ERROR_TOLERANCE2 ||
                        Math.abs(targetYDist) > ad2.POS_ERROR_TOLERANCE2)

                && opMode.opModeIsActive()) {



            odo.update();




            targetXDist =(targetX - ad2.getX());
            targetYDist = -(targetY - ad2.getY());
            totalDist = Math.hypot(targetXDist, targetYDist);


            opMode.telemetry.addData("Out X: " ,targetXDist );
            opMode.telemetry.addData("Out Y: " ,targetYDist );
            opMode.telemetry.update();
            ad2.telemetryd.addData("Out X: " ,targetXDist );
            ad2.telemetryd.addData("Out Y: " ,targetYDist );
            ad2.telemetryd.update();

            double currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);

            double angleToGo = getAngleToGo(getHeading(), currentTargetHead);

            double currentHeadingRad = Math.toRadians(getHeadingNorm());


            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = movePIDSpline(targetXDist,"y"); // Remember, Y stick value is reversed
            double x = movePIDSpline(targetYDist,"x");
            double rx = -turnPID(angleToGo);


            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
            double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            v1 = (rotY + rotX + rx) / denominator;
            v3 = (rotY - rotX + rx) / denominator;
            v2 = (rotY - rotX - rx) / denominator;
            v4 = (rotY + rotX - rx) / denominator;



            drive(v2,v4,v3,v1);
        }
    }

}
