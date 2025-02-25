package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Control implements Cloneable{
    public boolean LBumper1;
    public boolean RBumper1;

    public double LStickY;
    public double LStickX;
    public double RStickY;
    public double RStickX;

    public double LTrigger1; //
    public double RTrigger1; //
//

    public boolean a1;
    public boolean b1;
    public boolean x1;
    public boolean y1;

    public boolean a2;
    public boolean b2;
    public boolean x2;
    public boolean y2;

    public double LTrigger2;
    public double RTrigger2;
    public boolean LBumper2;
    public boolean RBumper2;

    public double RStickY2;
    public double RStickX2;
    public double LStickY2;
    public double LStickX2;

    public boolean dpadUp1;
    public boolean dpadDown1;
    public boolean dpadRight1;
    public boolean dpadLeft1;

    public boolean dpadUp2;
    public boolean dpadDown2;
    public boolean dpadRight2;
    public boolean dpadLeft2;

    public Gamepad gm1;
    public Gamepad gm2;

    public boolean options;
    public boolean options2;

    public boolean bigButton;
    public boolean bigButton2;

    public float padX;
    public float padY2;



    public Control(OpMode opMode) {
        gm1 = opMode.gamepad1;
        gm2 = opMode.gamepad2;

    }
    public void update(){

         LBumper1 = gm1.left_bumper;
         RBumper1 = gm1.right_bumper;
         LStickY = -gm1.left_stick_y;
         LStickX = -gm1.left_stick_x;
         RStickY = -gm1.right_stick_y;
         RStickX = gm1.right_stick_x;
         LTrigger1 = gm1.left_trigger;
         RTrigger1 = gm1.right_trigger;

         a1 = gm1.a;
         b1 = gm1.b;
         x1 = gm1.x;
         y1 = gm1.y;

         a2 = gm2.a;
         b2 = gm2.b;
         x2 = gm2.x;
         y2 = gm2.y;
         LTrigger2 = gm2.left_trigger;
         RTrigger2 = gm2.right_trigger;
         LBumper2 = gm2.left_bumper;
         RBumper2 = gm2.right_bumper;

        RStickY2 = -gm2.right_stick_y;
        RStickX2 = gm2.right_stick_x;
        LStickY2 = -gm2.left_stick_y;
        LStickX2 = gm2.left_stick_x;

        dpadUp1 = gm1.dpad_up;
        dpadDown1 = gm1.dpad_down;
        dpadRight1 = gm1.dpad_right;
        dpadLeft1 = gm1.dpad_left;

        dpadUp2 = gm2.dpad_up;
        dpadDown2 = gm2.dpad_down;
        dpadRight2 = gm2.dpad_right;
        dpadLeft2 = gm2.dpad_left;

        options = gm1.options;
        options2 = gm2.options;

        bigButton = gm1.touchpad;
        bigButton2 = gm2.touchpad;

        padX = gm1.touchpad_finger_1_x;
        padY2 = gm2.touchpad_finger_1_y;






    }

    @Override
    public Control clone() {
        try {
            return (Control)super.clone();
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException(e);
        }
    }
}
