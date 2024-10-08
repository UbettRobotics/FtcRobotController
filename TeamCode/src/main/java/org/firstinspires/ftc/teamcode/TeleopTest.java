package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Robot.*;

@TeleOp(name = "TelopTest")
public class TeleopTest extends LinearOpMode {

    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);


        CameraPipeline pipeline = new CameraPipeline(telemetry, 0,1);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            /*
             * This will be called if the camera could not be opened
             */

        });

//        initMotors(this);
//        initAUTO(this);
//        Control c = new Control(this);

        waitForStart();
        while(opModeIsActive()){
//            c.update();
//
//            rcDriving(c);
            Servo servoWheel = this.hardwareMap.get(Servo.class, "wheelServo");
            ColorSensor cs = this.hardwareMap.get(ColorSensor.class, "csi");
            DistanceSensor ds = ((DistanceSensor) cs);
//            Servo dumpServo = this.hardwareMap.get(Servo.class, "dumpServo");


            if(gamepad1.a){
                servoWheel.setPosition(1);
            } else if(gamepad1.b){
                servoWheel.setPosition(0);
            } else {
                servoWheel.setPosition(.5);
            }


            telemetry.addData("Distance: ", ds.getDistance(DistanceUnit.MM));
            telemetry.addData("R: ", cs.red());
            telemetry.addData("G: ", cs.green());
            telemetry.addData("B: ", cs.blue());
//            int[] encoderValues = ad.getEncoderPositions();
//            telemetry.addData("LEncoder: ", encoderValues[0]);
//            telemetry.addData("MEncoder: ", encoderValues[1]);
//            telemetry.addData("REncoder: ", encoderValues[2]);

//
//            telemetry.addData("X: ", imu.getANGLE(this, "X"));
//            telemetry.addData("Y: ", imu.getANGLE(this, "Y"));
//            telemetry.addData("Z: ", imu.getANGLE(this, "Z"));


            telemetry.update();

        }
    }
}
