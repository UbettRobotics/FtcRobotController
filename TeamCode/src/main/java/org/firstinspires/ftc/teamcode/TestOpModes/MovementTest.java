package org.firstinspires.ftc.teamcode.TestOpModes;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousDrive2;

@Autonomous(name = "Test Moves", group = "Test Modes")
public class MovementTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDrive2 ad2 = new AutonomousDrive2(this,false);

        initAccessories(this);

        int t = 1;

        double p = 0.02;
        double i = 0;
        double d = 0.006;

        int state  = 0;

        double diff = 0.0001;

        ad2.setOutputInfo(true);
        ad2.setTimeLimit(4);
        telemetry.addData("Pos X: ", ad2.getX());
        telemetry.addData("Pos Y: ", ad2.getY());
        telemetry.addData("Pos Heading: ", ad2.getHeading());
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            ad2.setPID(p,i,d,0);

            ad2.goToPointConstantHeading(24,24);
            ad2.goToPointConstantHeading(48,0);
            ad2.goToPointConstantHeading(0,0);
            telemetry.clearAll();
            telemetry.addData("P: ", p);
            telemetry.addData("I: ", i);
            telemetry.addData("D: ", d);
            telemetry.addData("State: ", state);
            telemetry.addData("Add or Minus: ", t);
            telemetry.update();
            double time = getRuntime();
            while(getRuntime() - time < 1 && opModeIsActive()) {
                    state = changeState(state);
                    if(gamepad1.y){
                        t = (t == 1)? -1:1;
                    }
                    telemetry.addData("P: ", p);
                    telemetry.addData("I: ", i);
                    telemetry.addData("D: ", d);
                    telemetry.addData("State: ", state);
                    telemetry.addData("Add or Minus: ", t);
                    telemetry.update();
            }
            switch (state){
                    case 1:
                        i+= diff * t;
                        break;
                    case 2:
                        d+= diff * t;
                        break;
                    default:
                        p+= diff * t;
                }

            }

            state = changeState(state);
            if(gamepad1.y){
                t = (t == 1)? -1:1;
            }

            switch (state){
                case 0:
                    p+= diff * t;
                    break;
                case 1:
                    i+= diff * t;
                    break;
                case 2:
                    d+= diff * t;
                    break;
            }


        }


    public int changeState(int state){
        if(gamepad1.a){
            state = 0;
        }else if(gamepad1.x){
            state = 1;
        }else if(gamepad1.b){
            state = 2;
        }

        return state;
    }






}

