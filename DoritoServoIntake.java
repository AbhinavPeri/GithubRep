/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.media.SoundPool;
import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;



import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Locale;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DoritoServoIntake", group="")
//@Disabled
public class DoritoServoIntake extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRearDrive, rightRearDrive, leftFrontDrive, rightFrontDrive, liftMotor = null;
    private Servo leftHook, rightHook, extakeServo1, extakeServo2 = null;
    private double armPosition, rightPower, leftPower;
    private boolean controlHook, controlGripper = true;

    //private double lRPower, rRPower, lFPower, rFPower;
    //private List<Double> powers = new ArrayList<>(4);

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRearDrive  = hardwareMap.dcMotor.get("leftRear");
        rightRearDrive = hardwareMap.dcMotor.get("rightRear");
        leftFrontDrive  = hardwareMap.dcMotor.get("leftFront");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        rightHook = hardwareMap.servo.get("rightHook");
        leftHook = hardwareMap.servo.get("leftHook");
        extakeServo1 = hardwareMap.servo.get("extakeServo1");
        extakeServo2 = hardwareMap.servo.get("extakeServo2");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //leftPower = gamepad1.left_stick_y - gamepad1.left_stick_x;
            //rightPower = gamepad1.left_stick_y + gamepad1.left_stick_x;

            //Range.clip(leftPower, 0, 1);
            //Range.clip(rightPower, 0,  1);

            /*leftFrontDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            leftRearDrive.setPower(leftPower);
            rightRearDrive.setPower(rightPower); */

            /*if (gamepad1.x){
                controlHook = false;
            }

            if (gamepad1.y){
                controlHook = true;
            }
            //moving the right hook
            if (controlHook){
                if (gamepad1.a){
                    rightHook.setPosition(0.7);
                    leftHook.setPosition(0.3);
                }else if (gamepad1.b){
                    rightHook.setPosition(0.3);
                    leftHook.setPosition(0.7);
                }else{
                    rightHook.setPosition(0.5);
                    leftHook.setPosition(0.5);
                }
            }else{
                rightHook.setPosition(0.3);
                leftHook.setPosition(0.7);
            }
            */
            //moving the lift
            armPosition += gamepad2.right_stick_y/1000;
            if (armPosition > 600){
                armPosition = 600;
            }else if (armPosition < 0){
                armPosition = 0;
            }
            liftMotor.setTargetPosition((int) armPosition);

            //liftMotor.setPower(gamepad2.right_stick_y * .8);
            if (gamepad2.x){
                controlGripper = false;
            }
            if (gamepad2.y){
                controlGripper = true;
            }
            if (controlGripper){
                if (gamepad2.a){
                    extakeServo1.setPosition(0.7);
                    extakeServo2.setPosition(0.1);
                }else if (gamepad2.b){
                    extakeServo1.setPosition(0.9);
                    extakeServo2.setPosition(0.3);
                }else {
                    extakeServo1.setPosition(0.5);
                    extakeServo2.setPosition(0.5);
                }
            }else{
                extakeServo1.setPosition(0.7);
                extakeServo2.setPosition(0.1);
            }


            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Lift Motor Ticks", liftMotor.getCurrentPosition());
            telemetry.update();




            // Show the elapsed game time and wheel power.
        }

    }
}
