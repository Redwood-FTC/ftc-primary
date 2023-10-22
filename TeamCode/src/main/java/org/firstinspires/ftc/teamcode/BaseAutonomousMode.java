package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Timer;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class BaseAutonomousMode extends LinearOpMode {
    public static double TILE_WIDTH = 24 * 24/21.5; // in
    private Servo purplePixelServo;
    private NormalizedColorSensor colorSensor;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        /* Setup for Roadrunner Drive */
        RobotOneMecanumDrive drive = new RobotOneMecanumDrive(hardwareMap);

        /* Camera Setup Start */
        initTfod();
        // X-value of line that separates left from center signal in camera image
        int leftCenterDivider = 250; // Robot 11.5cm from near tile interlocks
        float maxSignalDelay = 5000; // ms
        /* Camera Setup End */

        /* Color Sensor Setup Start */
        float gain = 3;
        final float[] hsvValues = new float[3];
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(gain);
        purplePixelServo = hardwareMap.get(Servo.class, "purple_pixel_servo");
        /* Color Sensor Setup End */


        waitForStart();

        long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        // Default to right signal
        String signal = new String("right");
        // Wait up to maxSignalDelay milliseconds before assuming the signal is out of view.
        while ((currentTime - startTime) < maxSignalDelay){
            currentTime = System.currentTimeMillis();
            /* Camera START */
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                if (x < leftCenterDivider){
                    signal = "left";
                } else {
                    signal = "center";
                }
                break;
            }
        }
        TrajectorySequenceBuilder toSignalTileTrajectoryBuilder = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(1 * TILE_WIDTH);
        Trajectory postSignalTrajectory;
        switch(signal){
            case "left":
                 toSignalTileTrajectoryBuilder
                         .strafeLeft(0.5 * TILE_WIDTH);
                break;
            case "right":
            default:
                toSignalTileTrajectoryBuilder
                        .strafeRight(0.5 * TILE_WIDTH);
                break;
            case "center":
                toSignalTileTrajectoryBuilder
                        .forward(0.5 * TILE_WIDTH);
                break;
        }
        drive.followTrajectorySequence(toSignalTileTrajectoryBuilder.build());

        while (!isStopRequested() && opModeIsActive()){
            telemetry.addData("Signal: ", signal);
            /* Camera END */

            /* Color sensor check START */
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            /* Logic to determine what color tape it is over. */
            String tapeColor = "None";
            float saturation = hsvValues[1];
            if (saturation >= 0.6) {
                if (colors.red > colors.blue) {
                    tapeColor = "Red";
                } else{
                    tapeColor = "Blue";
                }
            }
            telemetry.addData("Tape Color: ", tapeColor);
            /* Color sensor check END */

            if (gamepad1.a){
                purplePixelServo.setPosition(1);
            } else{
                purplePixelServo.setPosition(0);
            }
            //        drive.followTrajectory(trajectory);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName("team_props_1.tflite")
                .setModelLabels(new String[] {"b","r"})
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
    }   // end method initTfod()
}