package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sigma_Competition_Teleop", group = "LinearOpMode")
public class Sigma_Competition_Teleop extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(this, SkystoneDetector.Mode.ALTERNATESKYSTONE);
        robot.runTeleOp();

    }
}
