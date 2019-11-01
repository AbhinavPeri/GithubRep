package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Sigma_Competition_Teleop", group = " ")
public class Sigma_Competition_Teleop extends LinearOpMode{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor liftMotor = null;
    private Servo leftHook = null;
    private Servo rightHook = null;
    private boolean controlHook = true;
    private double lRPower, rRPower, lFPower, rFPower, liftPower;

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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            lFPower = gamepad1.left_stick_y - gamepad1.right_stick_x;
            rFPower = gamepad1.left_stick_y + gamepad1.right_stick_x;
            lRPower = gamepad1.left_stick_y - gamepad1.right_stick_x;
            rRPower = gamepad1.left_stick_y + gamepad1.right_stick_x;

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


            //moving the lift
            liftPower = -gamepad2.right_stick_y/3;
            liftMotor.setPower(liftPower);

            telemetry.addData("Controlling Hook", controlHook);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
            telemetry.addData("Left Stick X", gamepad1.right_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.right_stick_y);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
