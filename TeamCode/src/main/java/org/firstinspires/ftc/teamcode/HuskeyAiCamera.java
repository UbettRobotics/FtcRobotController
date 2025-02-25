package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import static org.firstinspires.ftc.teamcode.Robot.ad;
import static org.firstinspires.ftc.teamcode.Robot.c;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.AutonomousDrive.*;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;


public class HuskeyAiCamera {

    static HuskyLens huskyCam;

    static HuskyLens.Block[] blocks;

    public LinearOpMode opMode;

    public HuskeyAiCamera(LinearOpMode opMode){
        this.opMode = opMode;
        huskyCam = opMode.hardwareMap.get(HuskyLens.class, "huskeyCam");

        if (!huskyCam.knock()) {
            opMode.telemetry.addData(">>", "Problem communicating with " + huskyCam.getDeviceName());
        } else {
            opMode.telemetry.addData(">>", "Press start to continue");
        }
        opMode.telemetry.update();


    }

    public void changeAlgorthim(int num){
        switch(num){
            case(0):
                huskyCam.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
                break;
            case(1):
                huskyCam.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                break;
            case(2):
                huskyCam.selectAlgorithm(HuskyLens.Algorithm.FACE_RECOGNITION);
                break;
            case(3):
                huskyCam.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
                break;
        }
    }
    public static void updateCam(){
        blocks = (huskyCam.blocks().length > 0) ? huskyCam.blocks():new HuskyLens.Block[1];
    }

    public int getBlockCount(){
        updateCam();
        return blocks.length;
    }

    public ArrayList<int[]> getBlocksPos(){
        updateCam();
        ArrayList<int[]> poseList = new ArrayList<>();

        if(blocks.length > 0) {
            for(int i = 0; i < blocks.length; i++){
                poseList.add(new int[] {blocks[i].x,blocks[i].y});
            }
            return poseList;
        }else{
            return poseList;
        }
    }

    public void outPutInfo(LinearOpMode opMode){
        updateCam();
        if(blocks.length > 0) {

            for(int i = 0; i < blocks.length; i++){
                opMode.telemetry.addData("Block X " + i, blocks[i].x);
                opMode.telemetry.addData("Block Y " + i, blocks[i].y);
            }
            opMode.telemetry.update();

        }else{
            opMode.telemetry.addLine("There are no objects");
            opMode.telemetry.update();
        }
    }
}
