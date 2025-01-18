package org.firstinspires.ftc.teamcode.OpModes;


import static org.firstinspires.ftc.teamcode.Robot.ad;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Right - Connor", preselectTeleOp="Telop")

public class RightOne extends LinearOpMode  {


    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, false);

        //Prep Robot for Auto
        outtake.stopVSlide();
        outtake.setBucketPos(outtake.bucketRegPos);
        intake.tsTarget = intake.tsMiddle;
        intake.setTransferServo();

        outtake.closeClaw();


        waitForStart();
////////Program start////////////////////////////////////////////////////////////////////////

        // go to 70 y and 40 x
        ad.goToPointConstantHeading(40, 70); // get infront of chambers
        ad.goToHeading(0); // turn around
        outtake.vslideToPos(outtake.lowBucketSlidePos, 1); // raise slide
        sleep(2000);
        ad.forward(-5); // proceed into chamber

        outtake.killClaw(); // kill power to specimen claw
        sleep(50); // let specimen claw let go
        outtake.vslideToPos(outtake.touchBarSlidePos, 1); // pull specimen into chambers by lowering slide

        sleep(2000); // let slide get down
        ad.forward(5); // get away from chambers
        ad.goToHeading(180); // turn back around to forward-facing

        ad.goToPointConstantHeading(8.5, 120); // park?






    }
}
