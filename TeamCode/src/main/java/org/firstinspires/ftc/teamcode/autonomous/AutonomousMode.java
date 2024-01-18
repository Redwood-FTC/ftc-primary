package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Thread.sleep;

import android.graphics.Color;
import android.util.Log;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.fasterxml.jackson.databind.annotation.JsonAppend;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.RobotOneMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.DriveMode;
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
public class AutonomousMode extends DriveMode {
    protected enum Alliance {
        UNKNOWN,
        RED,
        BLUE
    }
//    protected Alliance ALLIANCE = Alliance.UNKNOWN;
    protected Alliance getAlliance() {
        return Alliance.UNKNOWN;
    }

    protected enum StartingPosition {
        UNKNOWN,
        FRONTSTAGE,
        BACKSTAGE
    }
    protected StartingPosition getStartingPosition() {
        return StartingPosition.UNKNOWN;
    }
    protected StartingPosition STARTING_POSITION = StartingPosition.UNKNOWN;

    protected enum PropPosition {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT
    }
    public static double TILE_WIDTH = 24 * 24/21.5; // inches
    private static double PIXEL_DROPPED = 1.0;
    private static double PIXEL_HOLDING = 0.2;
    public static double TURN_90 = -4.94;
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
//      Initialize the Apriltag Detection process
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
    private boolean once = false;
    @Override
    public void loop(){

        if (once) return;
        once = true;

        if (getStartingPosition() == StartingPosition.FRONTSTAGE) return;

        TrajectorySequenceBuilder toBoard = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(1 * TILE_WIDTH);

        while (teamPropPosition == PropPosition.UNKNOWN){
            // If we run out of time, assume the team prop is on the right.
            long timeSinceStart = System.currentTimeMillis() - startTime;
            if (timeSinceStart > maxSignalDelay){
                teamPropPosition = PropPosition.RIGHT;
                break;
            }
            /* Camera START */
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                if (x < leftCenterDivider){
                    teamPropPosition = PropPosition.LEFT;
                } else {
                    teamPropPosition = PropPosition.CENTER;
                }
                break;
            }
        }

        switch (teamPropPosition) {
            case LEFT:
                toBoard.turn(-TURN_90);
                break;
            case RIGHT:
                toBoard.turn(TURN_90);
                break;
            case CENTER:
            case UNKNOWN:
            default:
                break;
        }

        drive.setMotorPowers(0.1,0.1,0.1,0.1);

                // Creep forward until on the tape to drop the pixel.
        boolean onTape = false;
        Pose2d inTilePose = drive.getPoseEstimate();
        while (!onTape){
            /* Color sensor check START */
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            /* Logic to what color tape it is over. */
            float saturation = hsvValues[1];
            onTape = (saturation >= 0.6) || (colors.red > 0.04);
        }

        purplePixelServo.setPosition(PIXEL_DROPPED);

        drive.setMotorPowers(0, 0, 0, 0);

        //move back
        //reuse the code between switches
        switch (teamPropPosition) {
            case LEFT:
                break;
            case CENTER:
                break;
            case RIGHT:
                break;
            case UNKNOWN:
            default:
                break;
        }


        // dropPurplePixel(teamPropPosition);

        // Finished with TFOD, switching to AprilTag detector
//        initAprilTag(); //disabled until waiting for camera fixed
        // setManualExposure(6, 250); // Use low exposure time to reduce motion blur

        telemetry.addData("Signal:",teamPropPosition);
        TrajectorySequenceBuilder toSignalTileTrajectoryBuilder = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(1.7 * TILE_WIDTH); //go to middle of tile
        Trajectory postSignalTrajectory;
        switch (getAlliance()) { //use teampropposition to figure out how to turn
            case RED:
                telemetry.addLine("RED");
                toBoard.turn(TURN_90);
                break;
            case BLUE:
                telemetry.addLine("BLUE");
                toBoard.turn(-TURN_90);
                break;
            default:
                telemetry.addLine("default");
                break;
        }
        boolean targetFound = false;
        while (!targetFound) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }
        }
        Log.d("TARGET","TARGET FOUND");

        switch (teamPropPosition) {
            case LEFT:
//                toBoard.strafeLeft(0);
                break;
            case CENTER:
//                toBoard.strafeLeft(0);
                break;
            case RIGHT:
//                toBoard.strafeRight(0);
                break;
        }

        double startExtendTime = runtime.milliseconds();
        drive.followTrajectorySequence(toBoard.build());
        toBoard = drive.trajectorySequenceBuilder(new Pose2d());
        // Begin raising armAngleMotor
        armAngleMotor.setTargetPosition(5000);
        if ((runtime.milliseconds() - startExtendTime) > 1000) {
            wristServo.setPosition(0.76); // USE EXTENSION OF ARM MOTOR TO DETERMINE EXTENSION
        } // Separate if statement for separate tuning
        if ((runtime.milliseconds() - startExtendTime) > 1000) {
            armExtensionMotor.setTargetPosition(-1000); //was -2100
        }
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < 650);

        toBoard.forward(2.1 * TILE_WIDTH);
        switch(teamPropPosition){
            case LEFT:
                 toSignalTileTrajectoryBuilder
                         .turn(Math.PI / 2);
                 // Move left slightly
                break;
            case RIGHT:
            default:
//                toSignalTileTrajectoryBuilder
//                        .strafeRight(100);
                // Might move right
                break;
            case CENTER:
                // Move left slightly
                break;
        }
        // drive.followTrajectorySequence(toSignalTileTrajectoryBuilder.build());


        if (getStartingPosition() == StartingPosition.BACKSTAGE) {
            drive.followTrajectorySequence(toBoard.build());
        }

        bucketServo.setPosition(dropBucketPixelPosition);

        drive.followTrajectorySequence(toBoard.build());
        toBoard = drive.trajectorySequenceBuilder(new Pose2d());
        startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < 1500);

        toBoard.back(1 * TILE_WIDTH);
        drive.followTrajectorySequence(toBoard.build());
        toBoard = drive.trajectorySequenceBuilder(new Pose2d());
        startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < 650);
        armAngleMotor.setTargetPosition(0); //retract pixel arm
        armExtensionMotor.setTargetPosition(0);
        wristServo.setPosition(1.0);

        startTime = runtime.milliseconds(); //wait for retraction
        while (runtime.milliseconds() - startTime < 750);
        toBoard.forward(1 * TILE_WIDTH);

        drive.followTrajectorySequence(toBoard.build());
        toBoard = drive.trajectorySequenceBuilder(new Pose2d());

//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .waitSeconds(0.5)
//                        .lineToLinearHeading(inTilePose)
//                        .build()
//        );

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
    }   // End method initTfod()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        // Create the vision portal by using a builder.
        aprilTagVisionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
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

    protected void dropPurplePixel(PropPosition position) {
        purplePixelServo.setPosition(PIXEL_DROPPED);

//        boolean turnLeftToBackdrop = false;


        //purplePixelServo.setPosition(PIXEL_DROPPED);
        //purplePixelServo.setPosition(PIXEL_POST_DROP);

        // Move backwards to get out of way
    }
}
