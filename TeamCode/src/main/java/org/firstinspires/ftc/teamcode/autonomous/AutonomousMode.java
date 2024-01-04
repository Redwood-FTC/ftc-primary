package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.RobotOneMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.DriveMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Logger;
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
public class AutonomousMode extends DriveMode {
    protected enum Alliance {
        UNKNOWN,
        RED,
        BLUE
    }
    protected static Alliance ALLIANCE = Alliance.UNKNOWN;
    protected enum StartingPosition {
        UNKNOWN,
        FRONTSTAGE,
        BACKSTAGE
    }
    protected static StartingPosition STARTING_POSITION = StartingPosition.UNKNOWN;
    protected enum PropPosition {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT
    }
    private static double TILE_WIDTH = 24 * 24/21.5; // inches
    private static double PIXEL_DROPPED = 0.2;
    private static double PIXEL_HOLDING = 0.5;
    //private static double PIXEL_POST_DROP = 0.3;
    private Servo purplePixelServo;
    private NormalizedColorSensor colorSensor;
    private TfodProcessor tfod;
    private VisionPortal tfodVisionPortal;
    private VisionPortal aprilTagVisionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private final int leftCenterDivider = 250; // Robot 11.5cm from near tile interlocks
    private final float maxSignalDelay = 5000; // milliseconds
    private RobotOneMecanumDrive drive;
    private final float[] hsvValues = new float[3];
    private long startTime;
    @Override
    public void init() {
        drive = new RobotOneMecanumDrive(hardwareMap);
        super.init();
        /* Camera Setup Start */
        initTfod();
        // X-value of line that separates left from center signal in camera image
//      Initialize the Apriltag Detection process
//        initAprilTag();
//        setManualExposure(6, 250); // Use low exposure time to reduce motion blur
//        Camera Setup End

        float gain = 3;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        colorSensor.setGain(gain);
        purplePixelServo = hardwareMap.get(Servo.class, "purple_pixel_servo");
        purplePixelServo.setPosition(PIXEL_HOLDING);

    }
    private PropPosition teamPropPosition = PropPosition.UNKNOWN;
    @Override
    public void start(){
        super.start();
        startTime = System.currentTimeMillis();
    }

    private boolean purplePixelDropped = false;
    private boolean atBackdrop = false;
    @Override
    public void loop()  {
        while (teamPropPosition.equals(PropPosition.UNKNOWN)) {
            Logger.message("Looking for team prop position...");
            // If we run out of time, assume the team prop is on the right.
            long timeSinceStart = System.currentTimeMillis() - startTime;
            if (timeSinceStart > maxSignalDelay) {
                Logger.message("Ran out of time, assuming team prop is RIGHT.");
                teamPropPosition = PropPosition.RIGHT;
                break;
            }
            /* Camera START */
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                if (x < leftCenterDivider) {
                    Logger.message("Team prop identified at LEFT.");
                    teamPropPosition = PropPosition.LEFT;
                } else {
                    Logger.message("Team prop identified at CENTER.");
                    teamPropPosition = PropPosition.CENTER;
                }
                break;
            }
        }

         if (!purplePixelDropped) {
             dropPurplePixel();
             purplePixelDropped = true;
        }

         if (!atBackdrop){
            goToBackdrop();
            dropYellowPixel();
            atBackdrop = true;
        }



//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .waitSeconds(0.5)
//                        .lineToLinearHeading(inTilePose)
//                        .build()
//        );

        /* Color sensor check END */

//        int desired_tag_id = -1;
//        switch(signal){
//            case "left":
//                desired_tag_id = 4; // RED ALLIANCE TODO update for both alliances
//                break;
//            case "center":
//                desired_tag_id = 5;
//                break;
//            case "right":
//                desired_tag_id = 6;
//                break;
//        }
//        boolean targetFound = false;
//        while (!targetFound) {
//            // Step through the list of detected tags and look for a matching tag
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            for (AprilTagDetection detection : currentDetections) {
//                if ((detection.metadata != null)
//                        && (detection.id == desired_tag_id)){
//                    targetFound = true;
//                    desiredTag = detection;
//                    break; // don't look any further.
//                }
//            }
//        }
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
            while ((aprilTagVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

            ExposureControl exposureControl = aprilTagVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            GainControl gainControl = aprilTagVisionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
    }

    @Override
    protected void sendTelemetry(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        /* Logic to what color tape it is over. */
        float saturation = hsvValues[1];
        boolean onTape = (saturation >= 0.6) || (colors.red > 0.04);

        telemetry.addData("color sensor values 0: ", hsvValues[0]);
        telemetry.addData("color sensor values 1: ", hsvValues[1]);
        telemetry.addData("color sensor values 2: ", hsvValues[2]);
        telemetry.addData("saturation: ", saturation);
        telemetry.addData("onTape?: ", onTape);
        super.sendTelemetry();
    }

    private void dropPurplePixel() {
        Logger.message("Dropping purple pixel.");
        TrajectorySequenceBuilder toSignalTileTrajectoryBuilder = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(1 * TILE_WIDTH);
        Trajectory postSignalTrajectory;
        switch(teamPropPosition){
            case LEFT:
                toSignalTileTrajectoryBuilder
//                        .strafeLeft(1.2 * TILE_WIDTH);
                        .turn(Math.PI / 2);
                break;
            case RIGHT:
            default:
                toSignalTileTrajectoryBuilder
//                        .strafeRight(1.2 * TILE_WIDTH);
                        .turn(-Math.PI / 2);
                break;
            case CENTER:
                break;
        }
        drive.followTrajectorySequence(toSignalTileTrajectoryBuilder.build());
        purplePixelServo.setPosition(PIXEL_DROPPED);

//        boolean turnLeftToBackdrop = false;

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
//            TrajectoryBuilder partialTrajectory =
//                    drive.trajectoryBuilder(drive.getPoseEstimate())
//                            .forward(1);
//            if (turnLeftToBackdrop){
//                partialTrajectory.strafeLeft(1);
//            } else{
//                partialTrajectory.strafeRight(1);
//            }
//            drive.followTrajectory(
//                            partialTrajectory.build()
//            );
//            telemetry.addData("Saturation: ", saturation);
//            telemetry.update();
//        }
//
        //purplePixelServo.setPosition(PIXEL_DROPPED);
        //purplePixelServo.setPosition(PIXEL_POST_DROP);

        //move backwards to get out of way
    }

    private void goToBackdrop() {
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(drive.getPoseEstimate());
        switch (STARTING_POSITION){
            case BACKSTAGE:
                switch(ALLIANCE){
                    case RED:
                        switch(teamPropPosition){
                            case LEFT:
                                trajectorySequenceBuilder.turn(-Math.PI);
                                break;
                            case CENTER:
                                trajectorySequenceBuilder.turn(-Math.PI / 2);
                                break;
                        }
                        break;
                    case BLUE:
                        switch(teamPropPosition){
                            case RIGHT:
                                trajectorySequenceBuilder.turn(Math.PI);
                                break;
                            case CENTER:
                                trajectorySequenceBuilder.turn(Math.PI / 2);
                                break;
                        }
                        break;
                }
                trajectorySequenceBuilder.forward(1.5 * TILE_WIDTH);
                break;
        }

        drive.followTrajectorySequence(trajectorySequenceBuilder.build());
    }

    private void dropYellowPixel(){
        armAngleMotor.setTargetPosition(7200);
        long currentTime = System.currentTimeMillis();
        while (currentTime - System.currentTimeMillis() < 1000);
        wristServo.setPosition(0.76); // USE EXTENSION OF ARM MOTOR TO DETERMINE EXENSION
        armExtensionMotor.setTargetPosition(-2000);

        bucketServo.setPosition(dropBucketPixelPosition);
    }
}
