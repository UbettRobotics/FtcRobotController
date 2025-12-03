package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Outtake {
    public DcMotorEx launchMotor;


    public Servo loaderServo;

    public double launchPos = 0.25;
    public double loadingPos = 0;

    LinearOpMode opMode;

    double launchAngle = 58;

    double[] targetPos = new double[3];

    double wheelRadius = 1.41732;
    double ticksPerRev = 28;
    double launchSpeed = 1;

    double maxHeight = 60;

    double gravity = -388.826;//Gravity in inches/seconds^2

    public Outtake(LinearOpMode opMode, int sideNum){
        this.opMode = opMode;
        if(sideNum == 0){
            this.targetPos = targetPosBlue;
        }else{
            this.targetPos = targetPosRed;
        }

        launchMotor = opMode.hardwareMap.get(DcMotorEx.class, "launchMotor");
        loaderServo = opMode.hardwareMap.get(Servo.class, "loaderServo");
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void launchArt(){

        if(checkPose()){
            launchSpeed = 2400;
            launchMotor.setVelocity(3000);
            if(2400 <= launchMotor.getVelocity()){
                loaderServo.setPosition(launchPos);
            }else{
                loaderServo.setPosition(loadingPos);
            }

        }else{
            loaderServo.setPosition(loadingPos);
            launchMotor.setVelocity(0);
        }

    }

    public double getNewAngle(double speed){
        double angle = -2*gravity*maxHeight;
        angle = Math.sqrt(angle);
        angle = Math.asin(angle);
        return  Math.toDegrees(angle);
    }



    public double[] getLaunchSpeed(){
        odo.update();
        /*
        double distance = Math.hypot((ad2.getX()-targetPos[0]), (ad2.getY()-targetPos[1]));
        double u = Math.sqrt(-2*gravity*maxHeight);
        double time1 = (-u+Math.sqrt(-2 * gravity * (maxHeight + targetPos[1])))/gravity;
        double time2 = (-u-Math.sqrt(-2 * gravity * (maxHeight + targetPos[1])))/gravity;
        double time = Math.max(time1, time2);

        double voy = u;
        double vox = distance/time;
        double angle = Math.toDegrees(Math.atan(voy/vox));

        double speed = ((distance*distance)*gravity);
        speed /= (targetPos[2]-distance*Math.tan(Math.toRadians(launchAngle)))*(2*Math.pow(Math.cos(Math.toRadians(launchAngle)),2));
        speed = Math.sqrt(speed);
        double launchingSpeedTicks = convertToTickSpeed(speed);
        return new double[] {launchingSpeedTicks, angle};

         */

        double distance = Math.hypot((targetPos[0]-ad2.getX()), (targetPos[1] - ad2.getY()));
        double velo = Math.sqrt((gravity)/2/((targetPos[2]-distance*Math.tan(Math.toRadians(launchAngle)))));
        velo *= distance/(Math.cos(Math.toRadians(launchAngle)));
        velo *= 1.1;
        return new double[] {velo,launchAngle};



    }

    public boolean checkHeight(double speed){
        double dist = -(speed*speed)/(2 * gravity);
        return (dist >= maxHeight);

    }

    public double convertToTickSpeed(double speed){
        double tickSpeed = 0;
        tickSpeed = speed * ticksPerRev/(2 * Math.PI * wheelRadius) ;
        return tickSpeed;
    }

    public boolean checkPose(){
        boolean out = ((ad2.getX() >= Math.abs(ad2.getY()) - 7) || (ad2.getX() <= (-Math.abs(ad2.getY()) - 41)));

        return(out);
    }

    public void resetLoader(){
        loaderServo.setPosition(loadingPos);
    }

    public void stopLaunch(){
        launchMotor.setVelocity(0);
    }

    //returns the rpm
    public double getSpeedofLauncher(){
        return (launchMotor.getVelocity()/28)*60;
    }

    public void setLauncherRpm(double rpm){
        launchMotor.setVelocity(rpm/60*28);
    }



}
