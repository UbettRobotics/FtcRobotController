package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.teamcode.AutonomousDrive;
import org.firstinspires.ftc.teamcode.IMUControl;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "TelopTest")
public class TeleopTest extends LinearOpMode {

    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
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
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();

        waitForStart();
        while(opModeIsActive()){
            c.update();

            outtake.resetVSlide();


            if(c.a1){
                ascension.slidesToPow(.5);
            } else if (c.b1){
                ascension.slidesToPow(-.5);
            } else {
                ascension.slidesToPow(0);
            }
            ad.odo.update();

            Robot.rcDrivingFC();



            telemetry.addData("left encoder", ascension.leftMotor.getCurrentPosition());
            telemetry.addData("right encoder", ascension.rightMotor.getCurrentPosition());
            telemetry.addData("Gamepad pos X: ", c.padX);
            telemetry.addData("Gamepad pos Y: ", c.padY);
            telemetry.update();

        }
    }
}
