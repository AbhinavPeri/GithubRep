package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.*;

public class SigmaRobot{
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private Servo leftHook = null;
    private Servo rightHook = null;
    private double ticksPerInch = 44.5633840657;
    private BNO055IMU imu;

    public SigmaRobot() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()){

        }
    }


    public DcMotor getLeftRearDrive() {
        return leftRearDrive;
    }

    public void setLeftRearDrive(DcMotor leftRearDrive) {
        this.leftRearDrive = leftRearDrive;
    }

    public DcMotor getRightRearDrive() {
        return rightRearDrive;
    }

    public void setRightRearDrive(DcMotor rightRearDrive) {
        this.rightRearDrive = rightRearDrive;
    }

    public DcMotor getLeftFrontDrive() {
        return leftFrontDrive;
    }

    public void setLeftFrontDrive(DcMotor leftFrontDrive) {
        this.leftFrontDrive = leftFrontDrive;
    }

    public DcMotor getRightFrontDrive() {
        return rightFrontDrive;
    }

    public void setRightFrontDrive(DcMotor rightFrontDrive) {
        this.rightFrontDrive = rightFrontDrive;
    }

    public Servo getLeftHook() {
        return leftHook;
    }

    public void setLeftHook(Servo leftHook) {
        this.leftHook = leftHook;
    }

    public Servo getRightHook() {
        return rightHook;
    }

    public void setRightHook(Servo rightHook) {
        this.rightHook = rightHook;
    }

    public double getTicksPerInch() {
        return ticksPerInch;
    }

    public void setTicksPerInch(double ticksPerInch) {
        this.ticksPerInch = ticksPerInch;
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public void setImu(BNO055IMU imu) {
        this.imu = imu;
    }
}
