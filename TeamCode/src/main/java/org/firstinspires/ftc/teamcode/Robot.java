package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class Robot {


    public static double slow = 0.67;
    public static double fast = 1;

    public static DcMotorEx rf;
    public static DcMotorEx rb;
    public static DcMotorEx lb;
    public static DcMotorEx lf;

    public static DcMotorControllerEx dcMotorControllerEx;

    public static DistanceSensor distL, distR;


    public static Servo camServo;

    //public static HuskeyAiCamera huskCam;

    public static double gear = 1;
    public static boolean autoTurning = false;
    private static double gearprev = 1;


    public static GoBildaPinpointDriver odo;

    public static AutonomousDrive2 ad2;


    //Blue Side is 0 and Red is 1
    public static int side = 0;


    public static DcMotor[] motors = new DcMotor[4];



    public static void intAll(LinearOpMode opmode, int sideNum){
        initDrive(opmode);
        side = sideNum;

        odo = opmode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        //Pinpoint offsets from Center in mm
        //Left is +X, Front is +Y
        odo.setOffsets(-203,0,DistanceUnit.MM);

        odo.recalibrateIMU();
        opmode.sleep(200);
        odo.resetPosAndIMU();
        opmode.sleep(200);
        odo.setHeading(0, AngleUnit.DEGREES);

        ad2 = new AutonomousDrive2(opmode, odo);

        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0,0,AngleUnit.DEGREES, -90));



    }


    public static void initDrive(LinearOpMode opmode) {
        distL = opmode.hardwareMap.get(DistanceSensor.class, "distsensL");
        distR = opmode.hardwareMap.get(DistanceSensor.class, "distsensR");

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


        for (int i = 0; i < 4; i++) {
            dcMotorControllerEx.setMotorCurrentAlert(motors[i].getPortNumber(), 5, CurrentUnit.AMPS);
        }

        for (int i = 0; i < 4; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    //Getters and Setters and basic functions
    public static void drive(double rfPower, double rbPower, double lbPower, double lfPower) {

        rf.setPower(rfPower);
        rb.setPower(rbPower);
        lb.setPower(lbPower);
        lf.setPower(lfPower);
    }

    public static void driveFC(LinearOpMode opMode){
        odo.update();

        double targetX = odo.getPosY(DistanceUnit.INCH);
        double targetY = odo.getPosX(DistanceUnit.INCH);


        double angle = getRelativeTargetAngle(48-targetX, 48-targetY);





        double currentHeadingRad = Math.toRadians(odo.getHeading(AngleUnit.DEGREES));


        double v1 = 0;// lf
        double v2 = 0; // rf
        double v3 = 0; // lb
        double v4 = 0; // rb


        double y = -opMode.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = opMode.gamepad1.left_stick_x;
        double rx = 0;



        if(opMode.gamepad1.right_bumper){
            rx = Math.max(-0.3,Math.min(0.3,Robot.turnPID(Robot.getAngleToGo(odo.getHeading(AngleUnit.DEGREES)+180,angle))));
            opMode.telemetry.addData("angle to go: ", rx);
            opMode.telemetry.update();
        }else{
            rx = opMode.gamepad1.right_stick_x;
        }



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


        if(opMode.gamepad1.right_bumper||Math.abs(opMode.gamepad1.right_stick_x) >= 0.05 || Math.abs(opMode.gamepad1.left_stick_x) >= 0.05 || Math.abs(opMode.gamepad1.left_stick_y) >=0.05){
            Robot.drive(v2, v4, v3, v1);
        }else {
            Robot.drive(0,0,0,0);
        }
    }

    public static double getRelativeTargetAngle(double x, double y){
        double out = 0;
        out = Math.atan2(x,y);
        return  (Math.toDegrees(out) )% 360;
    }

    public static double turnPID(double error){
        double output = error * 0.0052 -odo.getHeadingVelocity(AngleUnit.DEGREES.getUnnormalized())*0.0005;
        return output;
    }

    public static double getAngleToGo(double currentHeading,double degrees){
        double angleTogo = degrees - currentHeading;

        if(Math.abs(angleTogo) > 180){
            if(currentHeading < 180){
                angleTogo = -((currentHeading) + (360 - degrees));
            }else{
                angleTogo = (degrees + (360 - currentHeading));
            }
        }
        return angleTogo;
    }

    public static void lineup(){
        double diff = distL.getDistance(DistanceUnit.INCH) - distR.getDistance(DistanceUnit.INCH);

        diff *= -10;
        double turn = turnPID(diff);

        drive(turn,turn, -turn,-turn);
    }




}












