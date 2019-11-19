package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Sigma_Competition_Teleop", group = " ")
public class Sigma_Competition_Teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRearDrive, rightRearDrive, leftFrontDrive, rightFrontDrive, liftMotor = null;
    private Servo leftHook, rightHook, extakeServo1, extakeServo2 , stopperServo, capstoneServo = null;
    private boolean controlHook = true;
    private double lRPower, rRPower, lFPower, rFPower, liftPower, armPosition, stopperPos, driverSensitivity, liftSensitivity, capstonePosition;

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
        //extakeServo1 = hardwareMap.servo.get("extakeServo1");
        //extakeServo2 = hardwareMap.servo.get("extakeServo2");
        //stopperServo = hardwareMap.servo.get("stopperServo");
        capstoneServo = hardwareMap.servo.get("capstoneServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controlHook = true;
        //extakeServo1.setDirection(Servo.Direction.REVERSE);
        capstonePosition = 0.5;
        driverSensitivity = 0.6;
        liftSensitivity = 0.5;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            lFPower = (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.right_trigger + gamepad1.left_trigger) * driverSensitivity;
            rFPower = (gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.right_trigger - gamepad1.left_trigger) * driverSensitivity;
            lRPower = (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.right_trigger - gamepad1.left_trigger) * driverSensitivity;
            rRPower = (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.right_trigger + gamepad1.left_trigger) * driverSensitivity;

            /*lFPower = Range.clip(lFPower, 0, 1);
            lRPower = Range.clip(lRPower, 0, 1);
            rRPower = Range.clip(rRPower, 0, 1);
            rFPower = Range.clip(rFPower, 0, 1); */

            leftFrontDrive.setPower(lFPower);
            leftRearDrive.setPower(lRPower);
            rightFrontDrive.setPower(rFPower);
            rightRearDrive.setPower(rRPower);

            if (gamepad1.x){
                controlHook = false;
            }

            if (gamepad1.y){
                controlHook = true;
            }
            //moving the foundation hooks
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
            if (gamepad1.right_bumper){
                driverSensitivity += 0.0003;
            }
            if (gamepad1.left_bumper){
                driverSensitivity -= 0.0003;
            }
            if (driverSensitivity < 0){
                driverSensitivity = 0;
            }
            if (driverSensitivity > 1){
                driverSensitivity = 1;
            }

            if (gamepad2.right_bumper){
                liftSensitivity += 0.0003;
            }
            if (gamepad2.left_bumper){
                liftSensitivity -= 0.0003;
            }
            if (liftSensitivity < 0){
                liftSensitivity = 0;
            }
            if (liftSensitivity > 1){
                liftSensitivity = 1;
            }

            armPosition += gamepad2.right_stick_y/2;
            if (armPosition > 600){
                armPosition = 600;
            }else if (armPosition < -600){
                armPosition = -600;
            }
            //liftMotor.setTargetPosition((int) armPosition);

            //moving the extake servo

            /*if (gamepad2.a){
                extakeServo1.setPosition(0.9);
                extakeServo2.setPosition(0.9);
            }else if (gamepad2.b) {
                extakeServo1.setPosition(0);
                extakeServo2.setPosition(0);
            }
             */

            if (gamepad1.dpad_left){
                capstonePosition = 1;
            }

            if (gamepad1.dpad_right){
                capstonePosition = 0.5;
            }

            liftMotor.setPower(gamepad2.left_stick_y * liftSensitivity);
            capstoneServo.setPosition(capstonePosition);
            //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //liftMotor.setPower(1);
            //Telemetry: displaying values on MotoPhone screen
            telemetry.addData("Capstone Position", capstonePosition);
            telemetry.addData("Stopper Position", stopperPos);
            telemetry.addData("Driver Sensitivity", driverSensitivity);
            telemetry.addData("Lift Sensitivity", liftSensitivity);
            //telemetry.addData("Lift Motor Ticks", liftMotor.getCurrentPosition());
            telemetry.addData("Arm Position", armPosition);
            telemetry.addData("Controlling Hook", controlHook);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    private int liftErrorTicks(int desiredTicks){
        return Math.abs(desiredTicks - liftMotor.getCurrentPosition());
    }

    private double liftPowerFunction(int errorTicks){
        double e = Math.E;
        return 1/(1 + Math.pow(e, (-1 * 0.05 * errorTicks)));
    }
}
