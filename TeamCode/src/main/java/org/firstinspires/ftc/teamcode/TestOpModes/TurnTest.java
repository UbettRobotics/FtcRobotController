package org.firstinspires.ftc.teamcode.TestOpModes;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousDrive2;

import static org.firstinspires.ftc.teamcode.Robot.*;

@Autonomous(name = "Turn Test", group = "Test Modes")
public class TurnTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initAccessories(this);
        AutonomousDrive2 ad2 = new AutonomousDrive2(this, true);


        int t = 1;

        double p = 0.018;
        double i = 0.0005;
        double d = 0.001;

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
            ad2.setPID(p,i,d,1);
            for(int k = 0; i < 8; k++){
                ad2.setPID(p,i,d,1);
                ad2.goToHeading(180 + k * 90);

                telemetry.clearAll();
                telemetry.addData("P: ", p);
                telemetry.addData("I: ", i);
                telemetry.addData("D: ", d);
                telemetry.addData("State: ", state);
                telemetry.addData("Add or Minus: ", t);
                telemetry.update();
                double time = getRuntime();
                while(getRuntime() - time < 1) {
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
