package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.sql.Driver;


public class Robot {


    public static final double slow = .65;
    public static final double fast = 1;

    public static DcMotor rf;
    public static DcMotor rb;
    public static DcMotor lb;
    public static DcMotor lf;


    public static ColorSensor cs1;

    public static Servo camServo;


    public static double gear = 1;
    private static double gearprev = 1;


    public static Control c;
    public static Control prevC;


    public static Intake intake;
    public static Outtake outtake;
    public static Ascension ascension;

    public static AutonomousDrive ad;
    public static IMUControl imu;
    //public static AprilTagPipeline aptag;

    public static DriverAutomation da;


    public static boolean foundBottom = false;



    public static void initDrive(OpMode opmode) {
        rf = opmode.hardwareMap.get(DcMotor.class, "rf");
        rb = opmode.hardwareMap.get(DcMotor.class, "rb");
        lb = opmode.hardwareMap.get(DcMotor.class, "lb");
        lf = opmode.hardwareMap.get(DcMotor.class, "lf");




        //0 ports is the back motors config and both motors are going to both control and expansion hub
        //Encoders go to those ports


        DcMotor[] motors = {rf, rb, lb, lf};
        for (int i = 0; i < 4; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    public static void initAccessories(OpMode opMode){
        //camServo = opMode.hardwareMap.get(Servo.class, "camservo");
        intake = new Intake(opMode);
        outtake = new Outtake(opMode);
        ascension = new Ascension(opMode);
        foundBottom = false;
    }


    public static void initAll(OpMode opMode){
        initAll(opMode, true);
    }
    public static void initAll(OpMode opMode, boolean left){

        initDrive(opMode);
        initAccessories(opMode);
//        imu = new IMUControl(opMode);

        c = new Control(opMode);
        c.update();

        prevC = (Control)c.clone();

        ad = new AutonomousDrive((LinearOpMode) opMode, left);
        da = new DriverAutomation();





        //Camera



        //aptag = new AprilTagPipeline((LinearOpMode) opMode);

    }





    public static void drive(double rfPower, double rbPower, double lbPower, double lfPower) {

        rf.setPower(rfPower * gear);
        rb.setPower(rbPower * gear);
        lb.setPower(lbPower * gear);
        lf.setPower(lfPower * gear);


    }



    public static double updateGear(){

        if((c.options && !prevC.options) || (c.options2 && !prevC.options2)){
            gearprev = (gearprev == slow) ? fast : slow;
        }
        gear = (intake.getCurrentHPos() > 500 || (outtake.vslide.getTargetPosition() > outtake.lowBucketSlidePos && outtake.getVSlidePos() > outtake.lowBucketSlidePos)) ? slow : gearprev;
        //gear = (outtake.getVSlidePos() > outtake.touchBarSlidePos) ? slow : gearprev;

        return gear;
    }


    public static double[] rcDriving(){

        updateGear();

        double v1 = 0; // lf
        double v2 = 0; // rf
        double v3 = 0; // lb
        double v4 = 0; // rb

        double sp = .25;



        if (Math.abs(c.LStickX) > 0.05 || Math.abs(c.LStickY) > 0.05 || Math.abs(c.RStickX) > 0.05) {

            if(intake.transferServo.getPosition() == intake.tsDown){
                c.RStickX = 0;
            }

            double r = Math.hypot(c.LStickX, c.LStickY) * gear;
            double robotAngle = Math.atan2(c.LStickY, c.LStickX) - Math.PI / 4;

            v1 = r * Math.sin(robotAngle) + c.RStickX; //lf // was cos
            v2 = r * Math.cos(robotAngle) - c.RStickX; //rf // was sin
            v3 = r * Math.cos(robotAngle) + c.RStickX; //lb // was sin
            v4 = r * Math.sin(robotAngle) - c.RStickX; //rb // was cos


        }

        else if (c.LTrigger1 > .25) {
            v1 = -sp;
            v2 = sp;
            v3 = sp;
            v4 = -sp;

        } else if (c.RTrigger1 > .25) {
            v1 = sp;
            v2 = -sp;
            v3 = -sp;
            v4 = sp;
        } else if (c.dpadUp1) {
            v1 = sp;
            v2 = sp;
            v3 = sp;
            v4 = sp;
        } else if (c.dpadRight1) {
            v1 = sp;
            v2 = -sp;
            v3 = -sp;
            v4 = sp;
        } else if (c.dpadDown1) {
            v1 = -sp;
            v2 = -sp;
            v3 = -sp;
            v4 = -sp;
        } else if (c.dpadLeft1) {
            v1 = -sp;
            v2 = sp;
            v3 = sp;
            v4 = -sp;
        } else if(c.RBumper1){
            v1 = sp;
            v2 = -sp;
            v3 = sp;
            v4 = -sp;
        }else if(c.LBumper1){
            v1 = -sp;
            v2 = sp;
            v3 = -sp;
            v4 = sp;
        }else {
            v1 = 0;
            v2 = 0;
            v3 = 0;
            v4 = 0;
        }


        //drive(v2, v4, v3, v1);
        return new double[] {v1, v2, v3, v4};



    }

    public static double[] rcDrivingFC(){

        updateGear();

        double v1 = 0; // lf
        double v2 = 0; // rf
        double v3 = 0; // lb
        double v4 = 0; // rb

        double sp = .25;
        ad.odo.update();



        if (Math.abs(c.LStickX) > 0 || Math.abs(c.LStickY) > 0 || Math.abs(c.RStickX) > 0) {

            double heading = -Math.toRadians(ad.odo.getHeading());
            double y = -c.LStickY;
            double x = c.LStickX * 1.1;
            double rx = c.RStickX;

            double feildX = x * Math.cos(heading) - y * Math.sin(heading);
            double feildY = x * Math.sin(heading) + y * Math.cos(heading);


            v1 = feildY + feildX + rx; //lf
            v2 =  feildY - feildX - rx; //rf
            v3 =  feildY - feildX + rx;  //lb
            v4 =  feildY + feildX - rx;  //rb

            double maxPow = Math.max(1.0, v1);
            maxPow = Math.max(maxPow, v2);
            maxPow = Math.max(maxPow, v3);
            maxPow = Math.max(maxPow, v4);

            v1 /= maxPow;
            v2 /= maxPow;
            v3 /= maxPow;
            v4 /= maxPow;

            Robot.drive(v2,v4,v3,v1);

        }else if (c.LTrigger1 > .25) {
            v1 = -sp;
            v2 = sp;
            v3 = sp;
            v4 = -sp;

        } else if (c.RTrigger1 > .25) {
            v1 = sp;
            v2 = -sp;
            v3 = -sp;
            v4 = sp;
        } else if (c.dpadUp1) {
            v1 = sp;
            v2 = sp;
            v3 = sp;
            v4 = sp;
        } else if (c.dpadRight1) {
            v1 = sp;
            v2 = -sp;
            v3 = -sp;
            v4 = sp;
        } else if (c.dpadDown1) {
            v1 = -sp;
            v2 = -sp;
            v3 = -sp;
            v4 = -sp;
        } else if (c.dpadLeft1) {
            v1 = -sp;
            v2 = sp;
            v3 = sp;
            v4 = -sp;
        } else if(c.RBumper1){
            v1 = sp;
            v2 = -sp;
            v3 = sp;
            v4 = -sp;
        }else if(c.LBumper1){
            v1 = -sp;
            v2 = sp;
            v3 = -sp;
            v4 = sp;
        }else {
            v1 = 0;
            v2 = 0;
            v3 = 0;
            v4 = 0;
        }


        drive(v2, v4, v3, v1);
        return new double[] {v2, v4, v3, v1};



    }

    public static void rcIntake(){
        //intake.resetHSlide();
        if(c.RBumper2 ){
            intake.runWheels(true);
        } else if(c.LBumper2){
            intake.runWheels(false);
        } else {
            intake.stopWheels();
        }

//        if(c.a2){
//            intake.tsTarget = intake.tsDown;
//        }else if(c.b2){
//            intake.tsTarget = intake.tsMiddle;
//        }else if(c.y2 && intake.hslide.getCurrentPosition() < 80){
//            intake.tsTarget = intake.tsUp;
//        }
        if(c.RStickY2 < -.5){
            intake.tsTarget = intake.tsDown;
        }else if(Math.abs(c.RStickX2) > .5 || c.RStickY2 > .5){
            intake.tsTarget = intake.tsMiddle;
        }
        intake.setTransferServo();




        if(c.LStickY2 > .05 && intake.getCurrentHPos() < intake.hSlideMax){
            intake.runSlide(c.LStickY2);
        } else if (c.LStickY2 < -.05) {
            intake.runSlide(c.LStickY2);
        }else {
            intake.stopSlide();
        }

    }

    //Outtake Control
    public static void rcOuttake(){


        //Slide Controls
        outtake.resetVSlide();
        if(!foundBottom && outtake.slideAtBottom()) {
            foundBottom = outtake.slideAtBottom();
            outtake.stopVSlide();
        }
        if(foundBottom) {
            if (c.dpadUp2) {
                //intake.tsTarget = intake.tsMiddle;
                outtake.targetPos = outtake.highBucketSlidePos;
            } else if (c.dpadLeft2) {
                //intake.tsTarget = intake.tsMiddle;
                outtake.targetPos = outtake.lowBucketSlidePos;
            } else if (c.dpadDown2) {
                outtake.killClaw();
                //intake.tsTarget = intake.tsMiddle;
                outtake.killClaw();
                outtake.targetPos = outtake.bottomSlidePos; //timing will need testing
            } else if (c.dpadRight2) {
                //intake.tsTarget = intake.tsMiddle;
                outtake.killClaw();
                outtake.targetPos = outtake.touchBarSlidePos;
            }


            if (!(outtake.targetPos == 0 && Math.abs(outtake.targetPos - outtake.getVSlidePos()) < 5)) {
                outtake.vslideToPos(outtake.targetPos, outtake.slidePower);
            } else {
                outtake.stopVSlide();
            }
        } else {
            if(c.dpadUp2){
                outtake.vslideToPow(outtake.slidePower);
            } else if(c.dpadDown2){
                outtake.vslideToPow(-outtake.slidePower);
            } else {
                outtake.stopVSlide();
            }
        }

        //Bucket Positions (for dumping)
        if (c.LTrigger2 > .10){
            outtake.targetBucketPos = outtake.bucketOutPos;
        } else {
            outtake.targetBucketPos = outtake.bucketRegPos;
        }

        if(c.a2){
            outtake.closeClaw();
        } else if(c.b2){
            outtake.openClaw();
        } else if(c.x2){
            outtake.killClaw();
        }

        outtake.setBucketPos(outtake.targetBucketPos);

    }

    public static void rcAscension(){
        if(c.bigButton && !prevC.bigButton){
            if(ascension.up){
                ascension.down();
            } else {
                ascension.up();
            }
        }


    }




}












