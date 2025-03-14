package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.DriverAutomation;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CameraPipeline;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Robot.*;

import java.sql.Driver;

@TeleOp(name = "Telop")
public class Teleop extends LinearOpMode {
    public int state;

    public CameraPipeline cam;



    @Override
    public void runOpMode() throws InterruptedException {

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
//
//
//        CameraPipeline pipeline = new CameraPipeline(telemetry, 0,1);
//        webcam.setPipeline(pipeline);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//
//            @Override
//            public void onError(int errorCode) {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
//            }
//            /*
//             * This will be called if the camera could not be opened
//             */
//
//        });

        initAll(this, true);


        int cameraMonitorViewId;

        OpenCvCamera webcam;


        cam = new CameraPipeline(telemetry);

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);

        webcam.setPipeline(cam);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            /*
             * This will be called if the camera could not be opened
             */

        });

        state = 0;

        waitForStart();
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();
        double[] power = new double[4];
        while(opModeIsActive()){
            c.update();
            state = da.update_auto_state(prevC, c, state);
            state = da.auto_intake_and_transfer(state, c, cam);
            power = rcDriving();

            rcIntake(state);
            rcOuttake();
            //rcAscension();



            Robot.drive(power[1],power[3],power[2],power[0]);

            //state = da.auto_intake_and_transfer(state);


            telemetry.addData("intake slide pos", intake.getCurrentHPos());
//            telemetry.addData("Intake Servo: ", intake.transferServo.getConnectionInfo());
            telemetry.addData("state: ", state);
            telemetry.addData("pady y 2", c.padY2);
            telemetry.addData("detect sample", cam.isDectedted());

//            telemetry.addData("color r", intake.cs.red());
//            telemetry.addData("color g", intake.cs.green());
//            telemetry.addData("color b", intake.cs.blue());
//            telemetry.addData("ds distance", intake.getDSDistance());
//            telemetry.addData("avg", (intake.cs.red() + intake.cs.green() + intake.cs.blue())/3.0);


            telemetry.update();


            prevC = c.clone();

        }
    }
}