package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class AutonomousDrive {


    public double errorTolerance = 0.015;

    public GoBildaPinpointDriver odo;

    public LinearOpMode opMode;


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
        opMode.sleep(150);
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
        double slope = 18;
        double max = .90;
        double min = .25;
        if(distanceToGo > 0) {
            return Math.max(Math.min(distanceToGo / slope, max), min);
        } else {
            return Math.min(Math.max(distanceToGo / slope, -max), -min);
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

    public void goToPoint(double targetX, double targetY, double degrees) {
        // try switching from atan2 to atan
        //Get X and Y Distance and Total Distance
        odo.update();

        double rotation = getHeading();
        double targetXDist = targetX - getX();
        double targetYDist= targetY - getY();
        double newY = -targetYDist * Math.cos(rotation) - -targetXDist * Math.sin(rotation); //Angle Difference Identity
        double newX = targetXDist * Math.cos(rotation) - -targetYDist * Math.sin(rotation); //Trigonometry

        //Get Heading
        degrees = (degrees >= 0) ? degrees % 360: (360 + (degrees % -360));
        double currentHeading = getHeading();
        double totalangleTogo = degrees - currentHeading;
        double startheading = getHeading();
        double angleTogo = totalangleTogo;
        currentHeading =  getHeading();
        angleTogo = degrees - currentHeading;
        if(Math.abs(angleTogo) > 180){
            if(currentHeading < 180){
                angleTogo = ((180-currentHeading) + degrees);
            }else{
                angleTogo = (degrees + (360 - currentHeading));
            }
        }
        double angleslope = angleTogo /Math.hypot(targetYDist,targetXDist);
        double powerTurn = powerCurvingTurn(angleTogo);

        double newdegree;




        double totalDistance = Math.hypot(newX, newY);
        double robotAngle = Math.atan2(targetYDist, targetXDist) + Math.PI/4 + Math.toRadians(angleTogo);
        double rightX = powerCurvingTurn(angleTogo);
        double power = powerCurvingOmni(totalDistance);

        double v1 = power * Math.cos(robotAngle) + rightX; //lf
        double v2 = power * Math.sin(robotAngle) - rightX; //rf
        double v3 = power * Math.sin(robotAngle) + rightX; //lb
        double v4 = power * Math.cos(robotAngle) - rightX; //rb

        Robot.drive(v2,v4,v3,v1);

        double angleToGo2 = angleTogo;
        while ((Math.abs(angleToGo2) > 5 || Math.abs(targetYDist) > this.errorTolerance + .025 || Math.abs(targetXDist) > this.errorTolerance + .025 ) && opMode.opModeIsActive()) {
            odo.update();

            rotation = getHeading();
            targetXDist = targetX - getX();
            targetYDist= targetY - getY();
            newY = -targetYDist * Math.cos(rotation) - -targetXDist * Math.sin(rotation); //Angle Difference Identity
            newX = targetXDist * Math.cos(rotation) - -targetYDist * Math.sin(rotation); //Trigonometry

            //Get Heading
            angleToGo2 = degrees - currentHeading;
            if (Math.abs(angleToGo2) > 180) {
                if (currentHeading < 180) {
                    angleToGo2 = ((180 - currentHeading) + degrees);
                } else {
                    angleToGo2 = (degrees + (360 - currentHeading));
                }
            }


            newdegree = startheading + (angleslope * totalDistance);
            angleTogo = newdegree - currentHeading;

            if (Math.abs(angleTogo) > 180) {
                if (currentHeading < 180) {
                    angleTogo = ((180 - currentHeading) + newdegree);
                } else {
                    angleTogo = (newdegree + (360 - currentHeading));
                }
            }

            totalDistance = Math.hypot(newX, newY);
            robotAngle = Math.atan2(targetYDist, targetXDist) + Math.PI/4 + Math.toRadians(angleTogo);
            rightX = powerCurvingTurn(angleTogo);
            power = powerCurvingOmni(totalDistance);

            v1 = power * Math.cos(robotAngle) + rightX; //lf
            v2 = power * Math.sin(robotAngle) - rightX; //rf
            v3 = power * Math.sin(robotAngle) + rightX; //lb
            v4 = power * Math.cos(robotAngle) - rightX; //rb

            Robot.drive(v2,v4,v3,v1);



            opMode.telemetry.addData("get x", getX());
            opMode.telemetry.addData("get y", getY());

            opMode.telemetry.addData("LF", v1);
            opMode.telemetry.addData("RF", v2);
            opMode.telemetry.addData("LB", v3);
            opMode.telemetry.addData("RB", v4);
            opMode.telemetry.update();
        }












        Robot.drive(0,0,0,0);

        opMode.sleep(350);
    }

    public void goToPointConstantHeading(double targetX, double targetY){
        // try switching from atan2 to atan
        //Get X and Y Distance and Total Distance
        odo.update();
        double targetYDistance = targetY - getY();
        double targetXDistance = targetX - getX();
        double totalDistance = Math.hypot(targetYDistance, targetXDistance);
        double angle = Math.atan2(targetYDistance, targetXDistance);

        //Get Heading
        double power;

        double v1; //lf // was cos
        double v2; //rf // was sin
        double v3; //lb // was sin
        double v4; //rb // was

        while((Math.abs(targetYDistance) > this.errorTolerance + .065 || Math.abs(targetXDistance) > this.errorTolerance + .065) && opMode.opModeIsActive()){
            odo.update();

            if(isStuck(totalDistance))return;

            targetYDistance = (targetY - odo.getPosition().getY(DistanceUnit.INCH));
            targetXDistance = (targetX - odo.getPosition().getX(DistanceUnit.INCH));
            totalDistance = Math.hypot(targetYDistance, targetXDistance);
            angle = Math.atan2(targetYDistance, targetXDistance) + Math.PI/4 + Math.toRadians(getHeading());


            power = powerCurvingOmni(totalDistance);


            v1 = power * Math.sin(angle); //lf // was cos
            v2 = power * Math.cos(angle); //rf // was sin
            v3 = power * Math.cos(angle); //lb // was sin
            v4 = power * Math.sin(angle); //rb // was


            if(Math.abs(v1) > .01 && Math.abs(v1) < .15) v1 = Math.abs(v1)/v1 * .2;
            if(Math.abs(v2) > .01 && Math.abs(v2) < .15) v2 = Math.abs(v1)/v1 * .2;
            if(Math.abs(v3) > .01 && Math.abs(v3) < .15) v3 = Math.abs(v1)/v1 * .2;
            if(Math.abs(v4) > .01 && Math.abs(v4) < .15) v4 = Math.abs(v1)/v1 * .2;

            if (v1 > 0 && v4 > 0 && v2 < 0 && v3 < 0){
//                v1 += .15;
//                v4 += .15;
                v1 *= 1.1;
                v2 *= 1.1;
                v3 *= 1.1;
                v4 *= 1.1;
            } else if (v1 < 0 && v4 < 0 && v2 > 0 && v3 > 0){
//                v2 += .15;
//                v3 += .15;
                v1 *= 1.1;
                v2 *= 1.1;
                v3 *= 1.1;
                v4 *= 1.1;
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
