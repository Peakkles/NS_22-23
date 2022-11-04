package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvWebcam;

public class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webcam;


    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");


    }
}