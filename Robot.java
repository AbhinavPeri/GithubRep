

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
        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Gamepad;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;












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















public class Robot {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double lRPower, rRPower, lFPower, rFPower, driverSensitivity, extensionPosition, capstonePosition, extensionServoPosition;
    private DcMotor leftRearDrive;
    private DcMotor rightRearDrive;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor extensionMotor;
    private Servo leftHook;
    private Servo rightHook;
    private Servo capstoneServo;
    private Servo extensionServo;
    private double ticksPerInch = 44.5633840657;
    private BNO055IMU imu;
    private boolean parkOnWall;
    private boolean controlHook = true;
    private LinearOpMode context;
    private HardwareMap hardwareMap;
    private Gamepad gamepad1, gamepad2;
    private Telemetry telemetry;
    private Detector detector;
    public void setParkOnWall(boolean T){

        parkOnWall = T;
    }

    public Robot(LinearOpMode opMode){

        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();
        context = opMode;
        this.hardwareMap = opMode.hardwareMap;
        gamepad1 = context.gamepad1;
        gamepad2 = context.gamepad2;
        telemetry = opMode.telemetry;
        detector = new Detector(opMode);
        detector.setMode(Detector.Mode.SKYSTONE);
        detector.startCamera();
        // Initialize the hardware variables. Note that the strings used here as parameters

        // to 'get' must correspond to the names assigned during the robot configuration

        // step (using the FTC Robot Controller app on the phone).

        leftRearDrive  = hardwareMap.get(DcMotor.class, "leftRear");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        extensionServo = hardwareMap.servo.get("extensionServo");
        capstoneServo = hardwareMap.servo.get("capstoneServo");
        rightHook = hardwareMap.servo.get("rightHook");
        leftHook = hardwareMap.servo.get("leftHook");

        capstoneServo.setPosition(0.5);

        driverSensitivity = 0.7;


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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()){

            if (context.isStopRequested()){
                break;
            }
        }

        extensionServoPosition = 0;
    }

    public int getBlueQuarryPositions(){
        double position = detector.getObject().getObjectLocation().y;
        try{
            if (position > 60 && position < 250){
                return 2;
            }
            if (position > 250 && position < 380){
                return 3;
            }

        }catch (Exception e){

        }
        return 1;
    }

    public int getRedQuarryPositions(){
        double position = detector.getObject().getObjectLocation().y;
        try{
            if ((position < 360 && position > 200)){
                return 2;
            }
            if ((position > 520 && position < 670) || (position > 50 && position < 200)){
                return 3;
            }
            if ((position > 360 && position < 520)){
                return 1;
            }

        }catch (Exception e){

        }
        return 2;

    }

    public void quarryRed(){
        runtime.reset();
        int position = 0;
        detector.setMode(Detector.Mode.SKYSTONE);
        detector.startCamera();
        while (context.isStarted() && !context.opModeIsActive()){
            try {
                position = getRedQuarryPositions();
            } catch (Exception e) {

            }
            if (!context.isStopRequested()){
                break;
            }
            telemetry.addData("Position", position);
            telemetry.update();
        }
        /*switch (position){
            case 1:
                quarryRed1();
                break;
            case 2:
                quarryRed2();
                break;
            case 3:
                quarryRed3();
                break;
        } */
        detector.closeCamera();
    }

    public void quarryRed1(){

        //move forward to make strafing easier
        moveInches(0.7, 2);

        //strafe left
        moveUsingEncoders(0.3, 11 * ticksPerInch, -11 * ticksPerInch, -11 * ticksPerInch, 11 * ticksPerInch);

        //collect skystone
        moveInches(0.7, 50);

        //turn to wall
        turnAccurate(imu, -175, 0.5);

        //go to wall
        moveInches(0.7, 47);

        //turn to skybridge
        turnAccurate(imu, -92, 0.5);

        //deposit stone
        moveInches(0.7, 60);

        //go back to retrieve the second stone
        moveInches(0.7, -73);

        //turn to collect next skystone
        turnAccurate(imu, 0, 0.5);

        //move the position 2 stone out of the way
        moveInches(0.7, 45);

        //move backwards to prepare for collecting the skystone
        moveInches(0.7, -8);

        //turn to collect the second stone
        turnAccurate(imu, 90, 0.5);

        //collect skystone
        moveInches(0.7, 14);

        turnAccurate(imu, -177, 0.5);

        moveInches(0.7, 37);

        turnAccurate(imu, -90, 0.5);

        moveInches(0.7, 80);

        moveInches(0.7, -15);
    }

    public void quarryRed2(){

        //Go forward to collect blocks
        moveInches(0.7, 50);

        //Turn 180 to face wall
        turnAccurate(imu, -179, 0.2);

        //Go to the wall
        if (parkOnWall){

            moveInches(0.7, 45);
        }else{

            moveInches(0.7, 22);
        }

        //Turn to face Skybridge
        turnAccurate(imu, -90, 0.1);

        //deliver Skystone
        moveInches(0.7, 54);

        //Go backwards to orient with the next Skystone
        if (parkOnWall){
            moveInches(0.7, -85);

        }else{
            moveInches(0.7, -83);

        }

        //Turn to face Skystone
        turnAccurate(imu, -5, 0.1);
        //
        //Collect Skystone the 2nd time
        //

        if (parkOnWall){

            moveInches(0.7, 48);
        }else{

            moveInches(0.7, 23);
        }

        //Turn 90 degrees
        turnAccurate(imu, -175, 0.1);

        //Go to the wall
        if (parkOnWall){

            moveInches(0.7, 52);
        }else{

            moveInches(0.7, 23);
        }

        //Turn to face Skybridge
        if (parkOnWall){

            turnAccurate(imu, -97, 0.1);
        }else{

            turnAccurate(imu, -90, 0.1);
        }

        //Deliver Skystone
        moveInches(0.7, 75);

        //Park
        moveInches(0.7, -15);
    }

    public void quarryRed3(){

        //Strafe right
        moveUsingEncoders(0.4, -15 * ticksPerInch, 15 * ticksPerInch, 15 * ticksPerInch, -15 * ticksPerInch);

        //Go forward to collect blocks
        moveInches(0.7, 50);

        //Turn 180 to face wall
        turnAccurate(imu, -179, 0.2);

        //Go to the wall
        if (parkOnWall){

            moveInches(0.7, 45);
        }else{

            moveInches(0.7, 22);
        }

        //Turn to face Skybridge
        turnAccurate(imu, -90, 0.1);

        //deliver Skystone
        moveInches(0.7, 54);

        //Go backwards to orient with the next Skystone
        if (parkOnWall){

            moveInches(0.7, -85);
        }else{

            moveInches(0.7, -83);
        }

        //Turn to face Skystone
        turnAccurate(imu, -5, 0.1);

        //
        //Collect Skystone the 2nd time
        //
        if (parkOnWall){

            moveInches(0.7, 48);
        }else{

            moveInches(0.7, 23);
        }

        //Turn 90 degrees
        turnAccurate(imu, -175, 0.1);

        //Go to the wall
        if (parkOnWall){

            moveInches(0.7, 52);
        }else{

            moveInches(0.7, 23);
        }

        //Turn to face Skybridge
        if (parkOnWall){

            turnAccurate(imu, -97, 0.1);
        }else{

            turnAccurate(imu, -90, 0.1);
        }

        //Deliver Skystone
        moveInches(0.7, 75);

        //Park
        moveInches(0.7, -15);
    }

    public void quarryBlue1(){

        //move forward to make strafing easier
        moveInches(0.7, 2);

        //strafe left



        moveUsingEncoders(0.3, -11 * ticksPerInch, 11 * ticksPerInch, 11 * ticksPerInch, -11 * ticksPerInch);



        //collect skystone



        moveInches(0.7, 50);



        //turn to wall



        turnAccurate(imu, 175, 0.5);



        //go to wall



        moveInches(0.7, 47);



        //turn to skybridge



        turnAccurate(imu, 92, 0.5);



        //deposit stone



        moveInches(0.7, 60);



        //go back to retrieve the second stone



        moveInches(0.7, -73);



        //turn to collect next skystone



        turnAccurate(imu, 0, 0.5);



        //move the position 2 stone out of the way



        moveInches(0.7, 45);



        //move backwards to prepare for collecting the skystone



        moveInches(0.7, -8);



        //turn to collect the second stone



        turnAccurate(imu, -90, 0.5);



        //collect skystone



        moveInches(0.7, 14);



        turnAccurate(imu, 177, 0.5);



        moveInches(0.7, 37);



        turnAccurate(imu, 90, 0.5);



        moveInches(0.7, 80);



        moveInches(0.7, -15);



    }







    public void quarryBlue2(){

        extensionServo.setPosition(0);

        //Go forward to collect blocks



        moveInches(0.7, 50);



        //Turn 180 to face wall




        turnAccurate(imu, 179, 0.2);



        //Go to the wall



        if (parkOnWall){



            moveInches(0.7, 45);



        }else{



            moveInches(0.7, 22);



        }



        //Turn to face Skybridge



        turnAccurate(imu, 90, 0.1);



        //deliver Skystone



        moveInches(0.7, 54);



        //Go backwards to orient with the next Skystone



        if (parkOnWall){



            moveInches(0.7, -85);



        }else{



            moveInches(0.7, -83);



        }



        //Turn to face Skystone



        turnAccurate(imu, 5, 0.1);



        //



        //Collect Skystone the 2nd time



        //



        if (parkOnWall){



            moveInches(0.7, 48);



        }else{



            moveInches(0.7, 23);



        }



        //Turn 90 degrees



        turnAccurate(imu, 175, 0.1);



        //Go to the wall



        if (parkOnWall){



            moveInches(0.7, 52);



        }else{



            moveInches(0.7, 23);



        }



        //Turn to face Skybridge



        if (parkOnWall){



            turnAccurate(imu, 97, 0.1);



        }else{



            turnAccurate(imu, 90, 0.1);



        }



        //Deliver Skystone



        moveInches(0.7, 75);



        //Park



        moveInches(0.7, -15);







    }







    public void quarryBlue3(){



        //Strafe right



        moveUsingEncoders(0.4, 10 * ticksPerInch, -10 * ticksPerInch, -10 * ticksPerInch, 10 * ticksPerInch);



        //Go forward to collect blocks



        moveInches(0.7, 50);



        //Turn 180 to face wall



        turnAccurate(imu, 180, 0.2);



        //Go to the wall



        moveInches(0.7, 45);



        //Turn to face Skybridge



        turnAccurate(imu, 90, 0.1);



        //deliver Skystone



        moveInches(0.7, 54);



        //Go backwards to orient with the next Skystone



        moveInches(0.7, -85);



        //Turn to face Skystone



        turnAccurate(imu, 5, 0.1);



        //



        //Collect Skystone the 2nd time



        //



        moveInches(0.7, 48);



        //Turn 90 degrees



        turnAccurate(imu, 175, 0.1);



        //Go to the wall



        if (parkOnWall){



            moveInches(0.7, 52);



        }else{



            moveInches(0.7, 28);



        }



        //Turn to face Skybridge



        turnAccurate(imu, 97, 0.1);



        //Deliver Skystone



        moveInches(0.7, 75);



        //Park



        moveInches(0.7, -20);



    }















    public void foundationBlue(){

        //move backwards, so the hooks are facing the foundation
        moveInches(0.3, -4);

        //strafe to the left, so that the robot is in the middle of the foundation edges
        moveUsingEncoders(0.3, ticksPerInch * -11.5 + 30, ticksPerInch * 11.5 + 30, ticksPerInch * 11.5 + 30,  ticksPerInch * -11.5 + 30);

        //move backwards to the foundation
        moveInches(0.7, -28);

        //move right hook downwards
        rightHook.setPosition(0.3);

        //move left hook downwards
        leftHook.setPosition(0.7);

        runtime.reset();

        while (runtime.time() < 2){

            if ( context.isStopRequested()){

                break;

            }

        }

        //move forwards to the wall, while holding the foundation with the grippers
        moveInches(0.4, 31);

        //Turn the foundation, so it goes in the building site
        turnAccurate(imu, 60, 0.1);

        //move a little bit more forward
        moveInches(.4, 2);

        //right hook goes up
        rightHook.setPosition(0.7);

        //left hook goes up
        leftHook.setPosition(0.3);

        while (runtime.time() < 2){

            if (context.isStopRequested()){

                break;

            }

        }

        moveUsingEncoders(0.3, ticksPerInch * 25 + 30, ticksPerInch * -25 + 30, ticksPerInch * -25 + 30,  ticksPerInch * 25 + 30);


        //if you want to park on the wall:
        if (parkOnWall){
            //go forward to parking spot
            moveInches(0.6, 16);

            //turn so robot is parallel with wall
            turnAccurate(imu,90, 0.1);

            //go forward a little
            moveInches(.4, 8);

            //strafe to right so robot is on wall
            moveUsingEncoders(.3, ticksPerInch * -10, ticksPerInch * 10, ticksPerInch * 10, ticksPerInch * -10);

            //if we want to park away from the wall:
        }else{

            //strafe further to parking spot to left
            moveUsingEncoders(0.3, ticksPerInch * 15 + 30, ticksPerInch * -15 + 30, ticksPerInch * -15 + 30,  ticksPerInch * 15 + 30);

            //turn towards parking spot
            turnAccurate(imu, 90, 0.1);

            //move forward a little
            moveInches(0.6, 10);





        }


        stopMotors();


    }







    public void foundationRed(){

        //move backwards with hooks facing foundation
        moveInches(0.3, -4);

        //Strafe to the right, so that the robot is more in the center of the foundation edges
        moveUsingEncoders(0.3, ticksPerInch * 11.5 + 60, ticksPerInch * -11.5 + 60, ticksPerInch * -11.5 + 60,  ticksPerInch * 11.5 + 60);

        //move backwards to foundation
        moveInches(0.7, -28);

        //move right hook downwards
        rightHook.setPosition(0.3);

        //move left hook downwards
        leftHook.setPosition(0.7);

        runtime.reset();

        while (runtime.time() < 2){

            if (context.isStopRequested()) {
                break;
            }
        }

        //move forwards while holding foundation with grippers
        moveInches(0.4, 31);

        //Turn the foundation, so it goes in the building site
        turnAccurate(imu, -60, 0.1);

        //move foundation more into building site
        moveInches(.4, 2);

        //moving right hook up
        rightHook.setPosition(0.7);

        //moving left hook up
        leftHook.setPosition(0.3);

        while (runtime.time() < 2){

            if (context.isStopRequested()){

                break;

            }

        }

        moveUsingEncoders(0.3, ticksPerInch * -25 + 30, ticksPerInch * 25 + 30, ticksPerInch * 25 + 30,  ticksPerInch * -25 + 30);


        //if you want to park on the wall:
        if (parkOnWall){
            //go forward to parking spot
            moveInches(0.6, 16);

            //turn so robot is parallel with wall
            turnAccurate(imu,-90, 0.1);

            //go forward a little
            moveInches(.4, 18);

            //strafe to right so robot is on wall
            moveUsingEncoders(.3, ticksPerInch * 10, ticksPerInch * -10, ticksPerInch * -10, ticksPerInch * 10);

            //if we want to park away from the wall:
        }else{

            //strafe further to parking spot to left
            moveUsingEncoders(0.3, ticksPerInch * -10 + 30, ticksPerInch * 150 + 30, ticksPerInch * 10 + 30,  ticksPerInch * -10 + 30);

            //turn towards parking spot
            turnAccurate(imu, -90, 0.1);

            //move forward a little
            moveInches(0.6, 20);

        }

        //stop the motors
        stopMotors();

    }







    private void stopMotors(){



        leftFrontDrive.setPower(0);



        leftRearDrive.setPower(0);



        rightFrontDrive.setPower(0);



        rightRearDrive.setPower(0);



    }







    private void moveUsingEncoders(double power, double leftRear, double leftFront, double rightRear, double rightFront){



        leftFront = Math.round(leftFront);



        leftRear = Math.round(leftRear);



        rightFront = Math.round(rightFront);



        rightRear = Math.round(rightRear);



        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);







        leftFrontDrive.setTargetPosition((int) -leftFront);



        leftRearDrive.setTargetPosition((int) -leftRear);



        rightFrontDrive.setTargetPosition((int) -rightFront);



        rightRearDrive.setTargetPosition((int) -rightRear);







        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);







        rightRearDrive.setPower(power);



        leftRearDrive.setPower(power);



        rightFrontDrive.setPower(power);



        leftFrontDrive.setPower(power);



        while (rightFrontDrive.isBusy() && leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightRearDrive.isBusy() && !context.isStopRequested()) {



            if (context.isStopRequested()){



                break;



            }



        }



        stopMotors();



    }







    public void moveInches(double power, double inches){



        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);







        leftFrontDrive.setTargetPosition((int) Math.round(-ticksPerInch * inches));



        leftRearDrive.setTargetPosition((int) Math.round(-ticksPerInch * inches));



        rightFrontDrive.setTargetPosition((int) Math.round(-ticksPerInch * inches));



        rightRearDrive.setTargetPosition((int) Math.round(-ticksPerInch * inches));







        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);







        rightRearDrive.setPower(power);



        leftRearDrive.setPower(power);



        rightFrontDrive.setPower(power);



        leftFrontDrive.setPower(power);



        while (rightFrontDrive.isBusy() && leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightRearDrive.isBusy() && !context.isStopRequested()) {



            if (context.isStopRequested()){



                break;



            }



        }



        stopMotors();



    }







    private void turnAccurate(BNO055IMU imu, double degrees, double threshold)



    {







        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        double initAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;



        double error = calculateAngularError(convertImuRegular(degrees), convertImuRegular(initAngle));



        double pow = sigmoidFunction1(error);



        double time = sigmoidFunction2(Math.abs(error));



        runtime.reset();



        while (error > threshold || error < -threshold && !context.isStopRequested()){











            rightRearDrive.setPower(pow);



            leftRearDrive.setPower(-pow);



            rightFrontDrive.setPower(pow);



            leftFrontDrive.setPower(-pow);







            error = calculateAngularError(convertImuRegular(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle), convertImuRegular(degrees));



            pow = sigmoidFunction1(error);



            if (runtime.time() > time){



                break;



            }



            if (context.isStopRequested()){



                break;



            }



        }



        stopMotors();



    }



    private double convertImuRegular(double angle){



        if (angle > 0){



            return 360 - angle;



        }else{



            return 0 + (-1 * angle);



        }



    }



    private double sigmoidFunction1(double error){



        double e = Math.E;



        return (1.4/(1 + Math.pow(e, -1 * (error/20.0)))) - 0.7;



    }







    private double sigmoidFunction2(double error){



        double e = Math.E;



        return (8/(1 + Math.pow(e, -1 * (error/65))) - 4);



    }







    private double calculateAngularError(double currentAngle, double targetAngle){



        //getting both distances



        double diff1 = currentAngle - targetAngle;



        double diff2;



        if (diff1 > 0){



            diff2 = (360 - diff1);



            diff1 *= -1;



        }else {



            diff2 = (360 + diff1) * -1;



            diff1 *= -1;



        }







        //seeing which one is bigger and returning the smaller one



        if (Math.abs(diff1) <= Math.abs(diff2)){



            return diff1;



        }else{



            return diff2;



        }



    }







    public void runTeleOp(){



        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        context.waitForStart();



        while (context.opModeIsActive()){



            lFPower = (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * driverSensitivity;



            rFPower = (gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * driverSensitivity;



            lRPower = (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * driverSensitivity;



            rRPower = (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * driverSensitivity;







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







            if (gamepad1.dpad_left){



                capstonePosition = 1;



            }







            if (gamepad1.dpad_right){



                capstonePosition = 0.5;



            }







            extensionPosition += (40 * gamepad1.right_trigger) - (40 * gamepad1.left_trigger);



            if (extensionPosition < 0){



                extensionPosition = 0;



            }







            if (extensionPosition > 4000){



                extensionPosition = 4000;



            }







            if (gamepad1.dpad_up){



                extensionServoPosition = 1;



            }



            if (gamepad1.dpad_down){



                extensionServoPosition = 0;



            }



            if (extensionServoPosition > 1){



                extensionServoPosition = 1;



            }



            if (extensionServoPosition < 0){



                extensionServoPosition = 0;



            }



            extensionServo.setPosition(extensionServoPosition);



            extensionMotor.setTargetPosition((int) extensionPosition);



            extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            extensionMotor.setPower(1);



            capstoneServo.setPosition(capstonePosition);



            telemetry.addData("Capstone Position", capstonePosition);



            telemetry.addData("Extension Servo Position", extensionServoPosition);



            telemetry.addData("Driver Sensitivity", driverSensitivity);



            telemetry.addData("Extension Position", extensionMotor.getCurrentPosition());



            telemetry.addData("Controlling Hook", controlHook);



            telemetry.addData("Status", "Run Time: " + runtime.toString());



            telemetry.update();



        }



    }











}





