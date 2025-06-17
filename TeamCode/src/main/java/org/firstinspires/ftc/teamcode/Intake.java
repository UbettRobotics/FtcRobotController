package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Intake {

    //Objects
    public DcMotorEx hslide;
//    public DcMotorEx hslide2;
    public Servo wheelServo;
    public Servo transferServo;
    public ColorSensor cs;
    public DistanceSensor ds;
    public TouchSensor hslideBottom;

    //Vars
    double inSlidePos = 0;
    double hSlideMax = 2400;

    public double tsDown = .3775;
    public double tsOutOfWay = .44;
    public double tsMiddle = 0.455;
    public double tsUp = tsMiddle;
    public double tsTarget = tsMiddle;

    public int slideOut = 2400;
    public int slideForceIn = -100;

    public DcMotorControllerEx motorControllerEx;

    private LinearOpMode opMode;


    public Intake(OpMode opMode){
        this.opMode = (LinearOpMode) opMode;

        hslide = opMode.hardwareMap.get(DcMotorEx.class, "hslide");
//        hslide2 = opMode.hardwareMap.get(DcMotorEx.class, "hslide2");

        wheelServo = opMode.hardwareMap.get(Servo.class, "servoWheelBlue");
        transferServo = opMode.hardwareMap.get(Servo.class, "servoTransferWhite");

        cs = opMode.hardwareMap.get(ColorSensor.class, "csi");
        ds = (DistanceSensor)cs;

        hslideBottom = opMode.hardwareMap.get(TouchSensor.class, "hslidelimit");
        hslide.setDirection(DcMotorSimple.Direction.REVERSE);

        hslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        hslide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        hslide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motorControllerEx = (DcMotorControllerEx)hslide.getController();


    }
    public int getCurrentHPos(){return hslide.getCurrentPosition();}
    public boolean slideAtBottom(){
        return hslideBottom.isPressed();
    }

    public void resetHSlide(){
        if(slideAtBottom()) {
            hslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            hslide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hslide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void runSlide(double power){
        hslide.setPower(power);

    }
    public void hslideToPow(double power){
        if(hslide.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            hslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            hslide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        hslide.setPower(power);
//        hslide2.setPower(power);
    }

    public void hslideToPos(int pos, double power){
        hslide.setTargetPosition(pos);
        hslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hslide.setPower(power);
        /*hslide2.setTargetPosition(pos);
        hslide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hslide2.setPower(power);*/
    }


    public void stopSlide(){
        hslide.setPower(0);
        //hslide2.setPower(0);
    }


    public void runWheels(boolean in){
        wheelServo.setPosition(in ? 0 : 1);
    }
    public void runWheels(boolean in, boolean slow){
        double dir = in ? 0 : 1;
//        double power = slow ? (5*dir + .5)/6 : dir;
        wheelServo.setPosition(dir);
    }
    public void stopWheels(){
        wheelServo.setPosition(.5);
    }

    public void setTransferServo() {
        transferServo.setPosition(tsTarget);
    }




   public SampleColor getColor(){
        if(cs.red() > 100) return SampleColor.RED;
        if(cs.blue() > 100) return SampleColor.BLUE;
        return SampleColor.YELLOW;
    }



    public boolean hasSample(){
        return ((intake.cs.red() + intake.cs.green() + intake.cs.blue())/3.0 < 20);
    }

    public double getDSDistance(){
        return ds.getDistance(DistanceUnit.INCH);
    }

    public void homeHSlide(){
        hslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startTime = opMode.time;
        while(motorControllerEx.getMotorCurrent(hslide.getPortNumber(), CurrentUnit.AMPS) < 9 && (opMode.time - startTime < 1)){
            hslide.setPower(-0.5);
            opMode.telemetry.addData("Hsilde Current: ", motorControllerEx.getMotorCurrent(hslide.getPortNumber(), CurrentUnit.AMPS));
        }
        hslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hslide.setPower(0);

    }







}
