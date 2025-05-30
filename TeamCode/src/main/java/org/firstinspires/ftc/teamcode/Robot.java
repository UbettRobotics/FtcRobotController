package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorHuskyLens;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.TestOpModes.MotorTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.sql.Driver;


public class Robot {


    public static double slow = 0.67;
    public static double fast = 1;

    public static DcMotorEx rf;
    public static DcMotorEx rb;
    public static DcMotorEx lb;
    public static DcMotorEx lf;

    public static DcMotorControllerEx dcMotorControllerEx;


    public static Servo camServo;

    public static HuskeyAiCamera huskCam;

    public static double gear = 1;
    private static double gearprev = 1;


    public static Control c;
    public static Control prevC;


    public static Intake intake;
    public static Outtake outtake;
    //public static Ascension ascension;

    public static AutonomousDrive ad;
    public static IMUControl imu;
    //public static AprilTagPipeline aptag;

    public static DriverAutomation da;

    public static DcMotor[] motors = new DcMotor[4];

    public static boolean foundBottom = false;




    public static void initDrive(OpMode opmode) {
        rf = opmode.hardwareMap.get(DcMotorEx.class, "rf");
        rb = opmode.hardwareMap.get(DcMotorEx.class, "rb");
        lb = opmode.hardwareMap.get(DcMotorEx.class, "lb");
        lf = opmode.hardwareMap.get(DcMotorEx.class, "lf");


        dcMotorControllerEx = (DcMotorControllerEx) (rf.getController());

        motors[0] = rf;
        motors[1] = rb;
        motors[2] = lb;
        motors[3] = lf;



        //0 ports is the back motors config and both motors are going to both control and expansion hub
        //Encoders go to those ports


        for(int i = 0; i < 4;i++){
            dcMotorControllerEx.setMotorCurrentAlert(motors[i].getPortNumber(), 5, CurrentUnit.AMPS);
        }

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
        //ascension = new Ascension(opMode);
        foundBottom = false;
        huskCam = new HuskeyAiCamera((LinearOpMode)opMode);
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

        rf.setPower(rfPower);
        rb.setPower(rbPower);
        lb.setPower(lbPower);
        lf.setPower(lfPower);


    }



    public static double updateGear(){

        if((c.options && !prevC.options)){
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

        double sp = .65;



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

    public static double[] rcDrivingFC(LinearOpMode opMode, double num){
        double sp = 0.5;

        double v1 = 0; // lf
        double v2 = 0; // rf
        double v3 = 0; // lb
        double v4 = 0; // rb




        if (Math.abs(c.LStickX) > 0 || Math.abs(c.LStickY) > 0 || Math.abs(c.RStickX) > 0) {

            double botHeading = -Math.toRadians(num);
            double y = -c.LStickY; // Remember, Y stick value is reversed
            double x = c.LStickX;
            double rx = c.RStickX;
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            v1 = (rotY + rotX + rx) / denominator;
            v3 = (rotY - rotX + rx) / denominator;
            v2 = (rotY - rotX - rx) / denominator;
            v4 = (rotY + rotX - rx) / denominator;




            Robot.drive(v2, v4, v3, v1);


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

    public static void rcIntake(int state){
        intake.resetHSlide();
        if(c.RBumper2 ){
            intake.runWheels(true);
        } else if(c.LBumper2){
            intake.runWheels(false);
        } else if (state != 4 && state != 9) {
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
            intake.tsTarget = intake.tsUp;
        }
        intake.setTransferServo();




        if(c.LStickY2 > .05 && intake.getCurrentHPos() < intake.hSlideMax){
            intake.hslideToPow(c.LStickY2);
        } else if (c.LStickY2 < -.05) {
            intake.hslideToPow(c.LStickY2);
        }else {
            if(intake.hslide.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                intake.stopSlide();
            }
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
//        if(c.bigButton && !prevC.bigButton){
//            if(ascension.up){
//                ascension.down();
//            } else {
//                ascension.up();
//            }
//        }


    }




}












