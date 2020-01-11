package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="AlternateQuarryRedAuto")


public class AlternateQuarryRedAuto extends LinearOpMode{

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, SkystoneDetector.Mode.ALTERNATESKYSTONE);
        robot.setParkOnWall(false);
        robot.runQuarry(Robot.Side.RED);
    }



}