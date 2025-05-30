package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.ArrayList;

public class MotorShell {
    public double motorMaxCurrent;
    public LynxModule controlHub, expansionHub;
    public ArrayList<Integer> motorNums = new ArrayList<>();
    public DcMotorControllerEx controllerEx;
    public LinearOpMode opMode;
    public final int motorRPM = 312;
    public final double tickPerRotation = 537.7;

    private double startTime = 0;
    public MotorShell(LinearOpMode opMode, double maxCurrent, LynxModule controlHub, LynxModule expansionHub, ArrayList<Integer> motorNums, DcMotorControllerEx controllerEx){
        motorMaxCurrent = maxCurrent;
        this.controlHub = controlHub;
        this.expansionHub = expansionHub;
        this.motorNums = motorNums;
        this.controllerEx = controllerEx;
        this.opMode = opMode;

        for(int i = 0; i < motorNums.size(); i++){
            controllerEx.setMotorMode(motorNums.get(i), DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public void drive(double rf, double rb, double lb, double lf){
        if(isCurrentAbove()){
            if(Math.abs(opMode.time - startTime) > 0.5) {
                for (int i = 0; i < motorNums.size(); i++) {
                    controllerEx.setMotorVelocity(motorNums.get(i), 0);
                }
                startTime = opMode.time;
            }
        }else{
            startTime = opMode.time;
            controllerEx.setMotorPower(motorNums.get(0), rf);
            controllerEx.setMotorPower(motorNums.get(1), rb);
            controllerEx.setMotorPower(motorNums.get(2), lb);
            controllerEx.setMotorPower(motorNums.get(3), lf);
        }
    }

    public boolean isCurrentAbove(){
        return (controlHub.getCurrent(CurrentUnit.AMPS) > 9.2);
    }



}
