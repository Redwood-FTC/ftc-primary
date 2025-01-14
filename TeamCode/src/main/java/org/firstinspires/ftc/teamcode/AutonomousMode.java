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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
* This is a simple routine to test translational drive capabilities.
*/
@Config
@Autonomous(name = "Autonomous Mode", group = "drive")
public class AutonomousMode extends LinearOpMode {
    public static double TILE_WIDTH = 24 * 24/21.5; // inches
    public static double PIXEL_DROPPED = -0.4000;
    public static double PIXEL_HOLDING = 0.1200;
    public static double PIXEL_POST_DROP = 0.3;
    private Servo purplePixelServo;
    private NormalizedColorSensor colorSensor;
    private TfodProcessor tfod;
    private VisionPortal tfodVisionPortal;
    private VisionPortal aprilTagVisionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        /* Setup for Roadrunner Drive */
        RobotOneMecanumDrive drive = new RobotOneMecanumDrive(hardwareMap);

        /* Camera Setup Start */
        initTfod();
        // X-value of line that separates left from center signal in camera image
        int leftCenterDivider = 250; // Robot 11.5cm from near tile interlocks
        float maxSignalDelay = 5000; // milliseconds
//      Initialize the Apriltag Detection process
//        initAprilTag();
//        setManualExposure(6, 250); // Use low exposure time to reduce motion blur
//        Camera Setup End

        // Color Sensor Setup Start
        float gain = 3;
        final float[] hsvValues = new float[3];
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(gain);
        purplePixelServo = hardwareMap.get(Servo.class, "purple_pixel_servo");
        purplePixelServo.setPosition(PIXEL_HOLDING);
        // Color Sensor Setup End

        waitForStart();

/**
 * example code:
 *       drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d())
 *           .strafeLeft(2 * TILE_WIDTH)
 *           .build());
 */

//        long startTime = System.currentTimeMillis();
//        long currentTime = System.currentTimeMillis();
//        // Default to right signal
//        String signal = "right";
//        // Wait up to maxSignalDelay milliseconds before assuming the signal is out of view.
//        while ((currentTime - startTime) < maxSignalDelay){
//            currentTime = System.currentTimeMillis();
//            /* Camera START */
//            List<Recognition> currentRecognitions = tfod.getRecognitions();
//            // Step through the list of recognitions and display info for each one.
//            for (Recognition recognition : currentRecognitions) {
//                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//                if (x < leftCenterDivider){
//                    signal = "left";
//                } else {
//                    signal = "center";
//                }
//                break;
//            }
//        }
//        TrajectorySequenceBuilder toSignalTileTrajectoryBuilder = drive.trajectorySequenceBuilder(new Pose2d())
//                .forward(1 * TILE_WIDTH);
//        Trajectory postSignalTrajectory;
//        switch(signal){
//            case "left":
//                 toSignalTileTrajectoryBuilder
//                         .turn(Math.PI / 2);
//                break;
//            case "right":
//            default:
//                toSignalTileTrajectoryBuilder
//                        .turn(-Math.PI / 2);
//                break;
//            case "center":
//                break;
//        }
//        drive.followTrajectorySequence(toSignalTileTrajectoryBuilder.build());
//
//        // Creep forward until on the tape to drop the pixel.
//        boolean onTape = false;
//        Pose2d inTilePose = drive.getPoseEstimate();
//        while (!onTape){
//            /* Color sensor check START */
//            NormalizedRGBA colors = colorSensor.getNormalizedColors();
//            Color.colorToHSV(colors.toColor(), hsvValues);
//            /* Logic to what color tape it is over. */
//            float saturation = hsvValues[1];
//            onTape = (saturation >= 0.6) || (colors.red > 0.04);
//            drive.followTrajectory(
//                    drive.trajectoryBuilder(drive.getPoseEstimate())
//                            .forward(1)
//                            .build()
//            );
//            telemetry.addData("Saturation: ", saturation);
//            telemetry.update();
//        }
//
//        sleep(500);
//        purplePixelServo.setPosition(PIXEL_DROPPED);
//        sleep(500);
//
////        for (int i = 0 ; i < 10; i++){
////            drive.setMotorPowers(1,1,1,1);
////            sleep(100);
////            drive.setMotorPowers(-1,-1,-1,-1);
////            sleep(100);
////        }
//
////        sleep(500); //make sure the pixel is on the ground before we set the servo
//        // It drags the pixel with it unless it's at a 90 degree angle to the ground
//        purplePixelServo.setPosition(PIXEL_POST_DROP);
//
//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .waitSeconds(0.5)
//                        .lineToLinearHeading(inTilePose)
//                        .build()
//        );
//
//        /* Color sensor check END */
//
////        int desired_tag_id = -1;
////        switch(signal){
////            case "left":
////                desired_tag_id = 4; // RED ALLIANCE TODO update for both alliances
////                break;
////            case "center":
////                desired_tag_id = 5;
////                break;
////            case "right":
////                desired_tag_id = 6;
////                break;
////        }
////        boolean targetFound = false;
////        while (!targetFound) {
////            // Step through the list of detected tags and look for a matching tag
////            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
////            for (AprilTagDetection detection : currentDetections) {
////                if ((detection.metadata != null)
////                        && (detection.id == desired_tag_id)){
////                    targetFound = true;
////                    desiredTag = detection;
////                    break; // don't look any further.
////                }
////            }
////        }
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
        tfodVisionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
    }   // end method initTfod()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        // Create the vision portal by using a builder.
        aprilTagVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Manually set the camera gain and exposure.
     * This can only be called AFTER calling initAprilTag(), and only works for Webcams.
     */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (aprilTagVisionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (aprilTagVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (aprilTagVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = aprilTagVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = aprilTagVisionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
