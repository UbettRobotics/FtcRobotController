package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {
    DcMotorEx intakeMotor;
    ColorSensor intakeColorSen;
    LinearOpMode opMode;
    double intakeSpeed = 0.67;

    public int normalizedRed = 0;
    public int normalizedGreen = 0;
    public int normalizedBlue = 0;
    public Intake(LinearOpMode opMode){
        this.opMode = opMode;
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeColorSen = opMode.hardwareMap.get(ColorSensor.class, "colorSenIntake");
    }

    //0 -- none
    //1 -- purple
    //2 -- green
    //0 -- stop
    //1 -- in
    //2 -- out
    public void intakeArt(int direction){
        if(direction == 0){
            intakeMotor.setPower(0);
        }else if(direction == 1) {
            intakeMotor.setPower(intakeSpeed);
        }else if(direction == 2) {
            intakeMotor.setPower(-0.75 * intakeSpeed);
        }
    }

    public int[] getColorIntake(){
        int r = intakeColorSen.red();
        int g = intakeColorSen.green();
        int b = intakeColorSen.blue();
        return new int[]{r,g,b};
    }

    public void normalizeColorVals(){
        int[] color = getColorIntake();
        normalizedRed = (normalizedRed + color[0])/2;
        normalizedGreen= (normalizedGreen + color[1])/2;
        normalizedBlue = (normalizedBlue + color[2])/2;
    }


    //Green --> 1
    //Purple -->0
    public int haveArtinIntake(){
        int[] color = getColorIntake();
        int red = color[0];
        int green = color[1];
        int blue = color[2];
        boolean[] colorStates = new boolean[]{red > normalizedRed + 10, green > normalizedGreen + 10, blue > normalizedBlue + 10};
        if(blue > green && blue > red && colorStates[2] && colorStates[1]){
            return 0;
        }else if(blue < green && green > red && colorStates[1]){
            return 1;
        }else{
            return -1;
        }
    }

}
