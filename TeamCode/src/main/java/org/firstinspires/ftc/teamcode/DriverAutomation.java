package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Robot.ad;
import static org.firstinspires.ftc.teamcode.Robot.c;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.AutonomousDrive.*;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriverAutomation {

    static long start_time = System.nanoTime();


    public static int phase = 0;

    public int auto_intake_and_transfer(int state, Control c, CameraPipeline cam){
        // states
        //        0 : inactive
        //        1 : extend slide
        //        2 : wait for slide
        //        3 : put transfer servo down and run intake
        //        4 : wait to get a sample
        //        5 : raise servo and stop intake
        //        6 : retract slide
        //        7 : wait for slide to retract
        //        8 : run intake
        //        9 : wait for thing to be intaked
        switch(state){
            case 1:
                intake.hslideToPos(intake.slideOut/2 + 200 +  (int)(1000 * c.padY2), 1.0);
                if(Math.abs(intake.getCurrentHPos() - (intake.slideOut/2 + 200 + (int)(1000*c.padY2))) < 100){
                    state = 2;
                }
                break;
            case 2:

                state = 3;
                break;
            case 3:
                intake.tsTarget = intake.tsDown;
                intake.setTransferServo();
                intake.runWheels(true);
                start_time = System.nanoTime();
                state = 4;
                break;
            case 4:
                if(this.getElapsedSeconds(start_time, System.nanoTime()) > 3 || cam.isDectedted()){
                    state = 5;
                }
                break;
            case 5:
                intake.tsTarget = intake.tsUp;
                intake.setTransferServo();
                intake.stopWheels();
                state = 6;
                break;
            case 6:
                intake.hslideToPos(0, 1);
                state = 7;
                break;
            case 7:
                if(intake.getCurrentHPos() < 50){
                    state = 8;
                }
                break;
            case 8:
                intake.runWheels(true);
                start_time = System.nanoTime();
                state = 9;
                break;
            case 9:
                if(getElapsedSeconds(start_time, System.nanoTime()) > 2.5){
                    state = 0;
                }
                break;


        }


        return state;

    }


    public double getElapsedSeconds(long startTime, long endTime) { // nano seconds
        return (endTime - startTime) / 1000000000.0;
    }

    public int update_auto_state(Control prevC, Control c, int state){
        if(!prevC.bigButton2 && c.bigButton2){
            if(state == 0){
                state = 1;
            } else {
                state = 0;
            }
        }
        if(!prevC.options2 && c.options2){
            if(state == 0){
                state = 3;
            } else {
                state = 0;
            }
        }
        return state;
    }
    public double[] cycleSample(LinearOpMode opMode){
        double[] pows = new double[4];
        if(c.a1) {
            //Phase
            //0:Start, retract intake and stop wheels
            //1: Go back and go into
            //
            //
            //
            //
            //
            //e
            switch(phase) {
                case 0:

                    if (intake.getCurrentHPos() < 100) {
                        phase++;
                    }
                    return new double[] {0,0,0,0};
                case 1:
                    pows = this.telegoToPointConstantHeading(opMode, 60, 24);
                    phase += (int)(pows[4]);
                    return new double[] {pows[0],pows[1],pows[2],pows[3]};
                case 2:
                    pows = this.telegoToHeading(opMode,90);
                    phase += (int)(pows[4]);
                    return new double[] {pows[0],pows[1],pows[2],pows[3]};
                case 3:
                    pows = this.telegoToPointConstantHeading(opMode, 14.5, 13);
                    if((int)(pows[4]) == 1){
                        phase++;
                    }
                    return new double[] {pows[0],pows[1],pows[2],pows[3]};



            }
        }else{
            phase = 0;
        }

        if(c.b1) {
            //Phase
            //0:Start, retract intake and stop wheels
            //1: Go back and go into
            //
            //
            //
            //
            //
            //e
            switch(phase) {
                case 0:

                    if (intake.getCurrentHPos() < 100) {
                        phase++;
                    }
                    return new double[] {0,0,0,0};
                case 1:
                    pows = this.telegoToPointConstantHeading(opMode, 60, 24);
                    phase += (int)(pows[4]);
                    return new double[] {pows[0],pows[1],pows[2],pows[3]};
                case 2:
                    pows = this.telegoToHeading(opMode,90);
                    phase += (int)(pows[4]);
                    return new double[] {pows[0],pows[1],pows[2],pows[3]};
                case 3:
                    pows = this.telegoToPointConstantHeading(opMode, 14.5, 13);
                    if((int)(pows[4]) == 1){
                        phase++;
                    }
                    return new double[] {pows[0],pows[1],pows[2],pows[3]};



            }
        }else{
            phase = 0;
        }
        return new double[4];

    }


    public double[] telegoToPointConstantHeading(LinearOpMode opMode, double targetX, double targetY){
        // try switching from atan2 to atan
        //Get X and Y Distance and Total Distance
        ad.odo.update();


        double targetYDistance = (targetY - ad.odo.getPosition().getY(DistanceUnit.INCH));
        double targetXDistance = (targetX - ad.odo.getPosition().getX(DistanceUnit.INCH));
        double totalDistance = Math.hypot(targetYDistance, targetXDistance);
        double angle = Math.atan2(targetYDistance, targetXDistance) + Math.PI/4 + Math.toRadians(ad.getHeading());
        if(ad.isStuck(totalDistance))return new double[]{0,0,0,0,1};

        if((Math.abs(targetYDistance) > ad.errorTolerance + .065 || Math.abs(targetXDistance) > ad.errorTolerance + .065) && opMode.opModeIsActive()){
            ad.odo.update();

            if(ad.isStuck(totalDistance))return new double[]{0,0,0,0,1};


            double power = powerCurvingOmni(totalDistance);


            double v1 = power * Math.sin(angle); //lf // was cos
            double v2 = power * Math.cos(angle); //rf // was sin
            double v3 = power * Math.cos(angle); //lb // was sin
            double v4 = power * Math.sin(angle); //rb // was


            if(Math.abs(v1) > .01 && Math.abs(v1) < .15) v1 = Math.abs(v1)/v1 * .2;
            if(Math.abs(v2) > .01 && Math.abs(v2) < .15) v2 = Math.abs(v1)/v1 * .2;
            if(Math.abs(v3) > .01 && Math.abs(v3) < .15) v3 = Math.abs(v1)/v1 * .2;
            if(Math.abs(v4) > .01 && Math.abs(v4) < .15) v4 = Math.abs(v1)/v1 * .2;

            if (v1 > 0 && v4 > 0 && v2 < 0 && v3 < 0){
                v1 += .15;
                v4 += .15;
            } else if (v1 < 0 && v4 < 0 && v2 > 0 && v3 > 0){
                v2 += .15;
                v3 += .15;
            }

            double[] out = {v1,v2,v3,v4,0.0};
            ad.outputDriveInfo(totalDistance, totalDistance, v1);

            return out;


        }




        double[] out = {0,0,0,0,1};
        return out;

    }

    public double[] telegoToHeading(LinearOpMode opMode, double degrees){
        if(degrees < 0) {
            degrees = (360 + (degrees % -360));
        }else{
            degrees = degrees % 360;
        }
        double currentHeading = ad.getHeading();
        double angleTogo = degrees - currentHeading;
        if(Math.abs(angleTogo) > 180){
            if(currentHeading < 180){
                angleTogo = -((currentHeading) + (360 - degrees));
            }else{
                angleTogo = (degrees + (360 - currentHeading));
            }
        }
        if(opMode.opModeIsActive() && Math.abs(angleTogo) > .05){

            double power =  powerCurvingTurn(angleTogo);

            opMode.telemetry.addData("Current Heading", currentHeading);
            opMode.telemetry.addData("Angle To go", angleTogo);
            opMode.telemetry.update();

            return new double[]{power,power,-power,-power,0};

        }
        return new double[]{0,0,0,0,1};
    }
}
