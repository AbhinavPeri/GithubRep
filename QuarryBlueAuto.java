package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;






@Autonomous(name="QuarryBlueAuto")


public class QuarryBlueAuto extends LinearOpMode{

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.setParkOnWall(false);
        robot.runQuarry(Robot.Side.BLUE);
    }



}