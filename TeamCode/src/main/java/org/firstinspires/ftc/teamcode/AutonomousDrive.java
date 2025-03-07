package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AutonomousDrive {


    public double errorTolerance = 0.015;

    public static GoBildaPinpointDriver odo;

    public LinearOpMode opMode;

    public static  double Integral = 0;



    /*
        180 is forward
        90 is left
        0/360 is backward
        270 is left


     */


    public AutonomousDrive(LinearOpMode opMode, boolean left) {

        this.opMode = opMode;
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(139.5, 101.6);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        double x = 8.5;
        double y = (left) ? 36 : 72;
        double heading = (left) ? 180 : 0;
        resetOdo((LinearOpMode)opMode, x, y, heading);
        ((LinearOpMode)opMode).sleep(250);


    }

    public void forward(double distanceTotal) {
        double startTime = this.opMode.time;
        double startpos = getX();
        double distanceToGo = distanceTotal;
        while (Math.abs(distanceToGo) > (this.errorTolerance + 0.13) && opMode.opModeIsActive()) {
            if(isStuck(distanceToGo)) return;

            distanceToGo = distanceTotal - (getX() - startpos);
            double power = powerCurving(distanceToGo);
            Robot.drive(power, power, power, power);
            this.outputDriveInfo(distanceToGo, distanceTotal, power);
            if(this.opMode.time - startTime > 4) break;

        }

//        while (Math.abs(distanceToGo) > this.errorTolerance + .12 && opMode.opModeIsActive()) {
//            if(isStuck(distanceToGo)) return;
//
//            distanceToGo = distanceTotal - (getX() - startpos);
//            double power = powerCurving(distanceToGo);
//            Robot.drive(power, power, power, power);
//            this.outputDriveInfo(distanceToGo, distanceTotal, power);
//            if(this.opMode.time - startTime > 5) break;
//
//        }

        Robot.drive(0, 0, 0, 0);

        opMode.sleep(75);
    }
    public void strafe(double distanceTotal){
        double startpos = getY();
        double distanceToGo = distanceTotal;
        while (Math.abs(distanceToGo) > (this.errorTolerance + 0.075) && opMode.opModeIsActive()) {
            if(isStuck(distanceToGo))return;
            distanceToGo = distanceTotal - (getY() - startpos);
            double power = powerCurving(distanceToGo);
            Robot.drive(-power, power, -power, power);
            this.outputDriveInfo( distanceToGo, distanceTotal, power);

        }

        while (Math.abs(distanceToGo) > this.errorTolerance + .025 && opMode.opModeIsActive()) {
            if(isStuck(distanceToGo)) return;
            distanceToGo = distanceTotal - (getY() - startpos);
            double power = powerCurving(distanceToGo) / 1.5;
            Robot.drive(-power, power, -power, power);
            this.outputDriveInfo(distanceToGo, distanceTotal, power);

        }

        Robot.drive(0, 0, 0, 0);
        opMode.sleep(150);

    }



    public void goToHeading(double degrees){
        if(degrees < 0) {
            degrees = (360 + (degrees % -360));
        }else{
            degrees = degrees % 360;
        }
        double currentHeading = getHeading();
        double angleTogo = degrees - currentHeading;
        while(opMode.opModeIsActive() && Math.abs(angleTogo) > .1){
            currentHeading =  getHeading();

            angleTogo = degrees - currentHeading;

            if(Math.abs(angleTogo) > 180){
                if(currentHeading < 180){
                    angleTogo = -((currentHeading) + (360 - degrees));
                }else{
                    angleTogo = (degrees + (360 - currentHeading));
                }
            }

            double power =  powerCurvingTurn(angleTogo);

            Robot.drive(power, power, -power, -power);



            opMode.telemetry.addData("Current Heading", currentHeading);
            opMode.telemetry.addData("Angle To go", angleTogo);
            opMode.telemetry.update();

        }
        Robot.drive(0, 0, 0, 0);
        opMode.sleep(25);
    }

    public void goToHeadingEvent(double degrees, double degreesLeftToEvent, event ev){
        boolean haveEvented = false;
        if(degrees < 0) {
            degrees = (360 + (degrees % -360));
        }else{
            degrees = degrees % 360;
        }
        double currentHeading = getHeading();
        double angleTogo = degrees - currentHeading;
        while(opMode.opModeIsActive() && Math.abs(angleTogo) > .5){
            currentHeading =  getHeading();

            angleTogo = degrees - currentHeading;

            if(Math.abs(angleTogo) <= degreesLeftToEvent && !haveEvented){
                ev.run();
                haveEvented = true;
            }

            if(Math.abs(angleTogo) > 180){
                if(currentHeading < 180){
                    angleTogo = -((currentHeading) + (360 - degrees));
                }else{
                    angleTogo = (degrees + (360 - currentHeading));
                }
            }

            double power =  powerCurvingTurn(angleTogo);

            Robot.drive(power, power, -power, -power);



            opMode.telemetry.addData("Current Heading", currentHeading);
            opMode.telemetry.addData("Angle To go", angleTogo);
            opMode.telemetry.update();

        }
        Robot.drive(0, 0, 0, 0);
        opMode.sleep(150);
    }

    public void goToHeadingEndEvent(double degrees, event ev){
        boolean haveEvented = false;
        if(degrees < 0) {
            degrees = (360 + (degrees % -360));
        }else{
            degrees = degrees % 360;
        }
        double currentHeading = getHeading();
        double angleTogo = degrees - currentHeading;
        while(opMode.opModeIsActive() && Math.abs(angleTogo) > .5){
            currentHeading =  getHeading();

            angleTogo = degrees - currentHeading;



            if(Math.abs(angleTogo) > 180){
                if(currentHeading < 180){
                    angleTogo = -((currentHeading) + (360 - degrees));
                }else{
                    angleTogo = (degrees + (360 - currentHeading));
                }
            }

            double power =  powerCurvingTurn(angleTogo);

            Robot.drive(power, power, -power, -power);



            opMode.telemetry.addData("Current Heading", currentHeading);
            opMode.telemetry.addData("Angle To go", angleTogo);
            opMode.telemetry.update();

        }
        Robot.drive(0, 0, 0, 0);
        ev.run();
        opMode.sleep(150);
    }

    public static double powerCurving(double distanceToGo){
        double slope = 18;
        double max = .80;
        double min = .2;
        if(distanceToGo > 0) {
            return Math.max(Math.min(distanceToGo / slope, max), min);
        } else {
            return Math.min(Math.max(distanceToGo / slope, -max), -min);
        }
    }

    public static double powerCurvingTurn(double angleToGo){
        double slope = 90;
        double min = .2;
        if(angleToGo > 0) {
            return (angleToGo / slope < min) ? min : angleToGo / slope;
        }else{
            return (angleToGo / slope > -min) ? -min : angleToGo / slope;
        }
    }

    public static double powerCurvingOmni(double distanceToGo){
        double slope = 24;
        double max = 0.9;
        double min = .20;
        double kD = 0.001;



        if(distanceToGo < 4){
            Integral += distanceToGo;
        }

        double speed = Math.hypot(odo.getVelocity().getX(DistanceUnit.INCH),odo.getVelocity().getY(DistanceUnit.INCH));
        speed = (distanceToGo > 0)? speed:-speed;
        if(distanceToGo > 0) {
            return Math.max( Math.min((distanceToGo/slope) - (speed * kD),max), min);
        } else {
            return Math.min(Math.min((distanceToGo/slope) - (speed * kD),max), -min);
        }


    }

    public double getX(){
        odo.update();
        return odo.getPosition().getX(DistanceUnit.INCH);

    }
    public double getY(){
        odo.update();
        return odo.getPosition().getY(DistanceUnit.INCH);
    }

    public double getHeading(){
        odo.update();
        //used to be 180 degrees
        return odo.getPosition().getHeading(AngleUnit.DEGREES) + 180;
    }



    public void resetOdo(LinearOpMode opMode, double x, double y, double heading){ // left is 8.5x and 36y // right is __x and __y
        odo.recalibrateIMU();
        opMode.sleep(500);
        odo.resetPosAndIMU();
        opMode.sleep(400);
        odo.setPosition(new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.DEGREES, heading));
    }

    public void setEstimatedPosition(double x, double y, double heading){
        odo.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
    }

    //Returns False if robot is not moving substantially
    public boolean isMoving(){
        return Math.abs(odo.getVelocity().getX(DistanceUnit.INCH)) > 0.005
                || Math.abs(odo.getVelocity().getY(DistanceUnit.INCH)) > 0.005
                ||  Math.abs(odo.getVelocity().getHeading(AngleUnit.DEGREES)) > 0.005;
    }
    public boolean isStuck(double distanceToGo){
        return  !isMoving() && Math.abs(distanceToGo) < 1.75;
    }

    public static double getNewDegree(double targetDegree, double totalheading, double totalDist, double dist){
        double newdegree = 0;
        double slope = totalheading/totalDist;
        newdegree = targetDegree - (totalheading -  (dist * slope));
        return newdegree;
    }

    public void goToPoint(double targetX, double targetY, double degrees, LinearOpMode opMode) {
        double targetYDistance = (targetY - getY()) * 1.1;
        double targetXDistance = targetX - getX();
        double totalDistance = Math.hypot(targetYDistance, targetXDistance);

        //Get Heading
        double botHeading = getHeading();
        if(degrees < 0) {
            degrees = (360 + (degrees % -360));
        }else{
            degrees = degrees % 360;
        }

        double angleTogo = degrees - botHeading;



        if(Math.abs(angleTogo) > 180){
            if(botHeading < 180){
                angleTogo = -((botHeading) + (360 - degrees));
            }else{
                angleTogo = (degrees + (360 - botHeading));
            }
        }

        double totalheadingangle = angleTogo;
        double totaldist = totalDistance;


        double power;
        double slope = 90;
        double min = .25;




        double v1; //lf // was cos
        double v2; //rf // was sin
        double v3; //lb // was sin
        double v4; //rb // was

        double newDegree = getNewDegree(degrees,totalheadingangle, totaldist,totalDistance);
        double newangleToGo= botHeading - newDegree;

        double rx;

        while((Math.abs(angleTogo) > 0.5 ||Math.abs(targetYDistance) > this.errorTolerance + .75 || Math.abs(targetXDistance) > this.errorTolerance + .75) && opMode.opModeIsActive()){
            odo.update();


            targetYDistance = (targetY - odo.getPosition().getY(DistanceUnit.INCH));
            targetXDistance = (targetX - odo.getPosition().getX(DistanceUnit.INCH));
            totalDistance = Math.hypot(targetYDistance, targetXDistance);
            botHeading = getHeading();





            power = powerCurvingOmni(totalDistance);
            double botheadingRad = -Math.toRadians(odo.getHeading());

            // Rotate the movement direction counter to the bot's rotation
            double rotX = targetYDistance * Math.cos(-botheadingRad) - targetXDistance * Math.sin(-botheadingRad);
            double rotY = targetYDistance * Math.sin(-botheadingRad) + targetXDistance * Math.cos(-botheadingRad);


            //get target Heading

            newDegree = getNewDegree(degrees,totalheadingangle,  totaldist,totalDistance);

            if(Math.abs(angleTogo) > 180){
                if(botHeading < 180){
                    angleTogo = -((botHeading) + (360 - degrees));
                }else{
                    angleTogo = (degrees + (360 - degrees));
                }
            }

            angleTogo = degrees - botHeading;
            if(Math.abs(newangleToGo) > 180){
                if(botHeading < 180){
                    newangleToGo = -((botHeading) + (360 - newDegree));
                }else{
                    newangleToGo = (newDegree + (360 - newDegree));
                }
            }

            if(newangleToGo > 0) {
                 rx =  (newangleToGo / slope < min) ? min : newangleToGo / slope;
            }else{
                 rx = (newangleToGo / slope > -min) ? -min : newangleToGo / slope;
            }


            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            v1 = (rotY + rotX + rx) / denominator;
            v3 = (rotY - rotX + rx) / denominator;
            v2 = (rotY - rotX - rx) / denominator;
            v4 = (rotY + rotX - rx) / denominator;


         /*   double max = 1;
            max = Math.max(v1, max);
            max = Math.max(v2, max);
            max = Math.max(v3, max);
            max = Math.max(v4, max);

            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;

          */

            opMode.telemetry.addData("X: ", getX());
            opMode.telemetry.addData("Y: ", getY());
            opMode.telemetry.addData("heading: ", getHeading());
            opMode.telemetry.addData("Target degree: ", newangleToGo);
            opMode.telemetry.update();
            Robot.drive(v2,v4,v1,v3);
        }

            Robot.drive(0, 0, 0, 0);

            opMode.sleep(100);

    }

    public void goToPointConstantHeading(double targetX, double targetY){
        // try switching from atan2 to atan
        //Get X and Y Distance and Total Distance
        odo.update();
        double targetYDistance = (targetY - getY()) * 1.1;
        double targetXDistance = targetX - getX();
        double totalDistance = Math.hypot(targetYDistance, targetXDistance);
        double angle = Math.atan2(targetYDistance, targetXDistance);

        //Get Heading
        double power;

        double v1; //lf // was cos
        double v2; //rf // was sin
        double v3; //lb // was sin
        double v4; //rb // was

        while((Math.abs(targetYDistance) > this.errorTolerance + .75 || Math.abs(targetXDistance) > this.errorTolerance + .75) && opMode.opModeIsActive()){
            odo.update();

            if(isStuck(totalDistance))return;

            targetYDistance = (targetY - odo.getPosition().getY(DistanceUnit.INCH)) * 1.1;
            targetXDistance = (targetX - odo.getPosition().getX(DistanceUnit.INCH));
            totalDistance = Math.hypot(targetYDistance, targetXDistance);
            angle = Math.atan2(targetYDistance, targetXDistance) + Math.PI/4 + Math.toRadians(getHeading());


            power = powerCurvingOmni(totalDistance);


            v1 = power * Math.sin(angle); //lf // was cos
            v2 = power * Math.cos(angle); //rf // was sin
            v3 = power * Math.cos(angle); //lb // was sin
            v4 = power * Math.sin(angle); //rb // was


//            if(Math.abs(v1) > .01 && Math.abs(v1) < .15) v1 = Math.abs(v1)/v1 * .2;
//            if(Math.abs(v2) > .01 && Math.abs(v2) < .15) v2 = Math.abs(v1)/v1 * .2;
//            if(Math.abs(v3) > .01 && Math.abs(v3) < .15) v3 = Math.abs(v1)/v1 * .2;
//            if(Math.abs(v4) > .01 && Math.abs(v4) < .15) v4 = Math.abs(v1)/v1 * .2;

            double scale_factor = 1.5;
            if (v1 > 0 && v4 > 0 && v2 < 0 && v3 < 0){
//                v1 += .15;
//                v4 += .15;
                v1 *= scale_factor;
                v2 *= scale_factor;
                v3 *= scale_factor;
                v4 *= scale_factor;
            } else if (v1 < 0 && v4 < 0 && v2 > 0 && v3 > 0){
//                v2 += .15;
//                v3 += .15;
                v1 *= scale_factor;
                v2 *= scale_factor;
                v3 *= scale_factor;
                v4 *= scale_factor;
            }

            Robot.drive(v2,v4,v3,v1);

            outputDriveInfo(totalDistance, totalDistance, v1);
        }




        Robot.drive(0,0,0,0);

        opMode.sleep(150);



    }








    public void outputDriveInfo(double distanceToGo, double distanceTotal, double power){
//        opMode.telemetry.addData("distanceToGo", distanceToGo);
//        opMode.telemetry.addData("distanceTotal", distanceTotal);
//        opMode.telemetry.addData("power", power);
////        opMode.telemetry.addData("get x", this.getX());
////        opMode.telemetry.addData("get y", this.getY());
//        opMode.telemetry.addData("heading ", odo.getPosition().getHeading(AngleUnit.DEGREES));
//        opMode.telemetry.addData("is moving", isMoving());
//        opMode.telemetry.update();
        opMode.telemetry.addData("get x", this.getX());
        opMode.telemetry.addData("get y", this.getY());
        opMode.telemetry.addData("v1 power", power);
        opMode.telemetry.addData("total distance", distanceToGo);
        opMode.telemetry.update();
    }


}
