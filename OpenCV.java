package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OpenCV", group = "LinearOpMode")
public class OpenCV extends LinearOpMode {
    Detector detector;
    Object object;

    @Override
    public void runOpMode() throws InterruptedException {
        //detector = new Detector(this, "Detector2.properties");
        detector = new Detector(this);
        detector.setMode(Detector.Mode.BLUEFOUNDATION);
        waitForStart();
        detector.startCamera();
        while (opModeIsActive()){

        }
        detector.closeCamera();
    }
}
