package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.autonomous.RobotDrive;
import org.firstinspires.ftc.teamcode.autonomous.RobotDrive.*;

public class Pixel {
    private AutonomousMode.PropPosition teamPropPosition;
    private RobotDrive drive;
    private NormalizedColorSensor colorSensor;
    private Servo purplePixelServo;
    private DcMotor armAngleMotor;
    private DcMotor armExtensionMotor;
    private Servo bucketServo;
    private Servo wristServo;
    private final float[] hsvValues = new float[3];
    private double PIXEL_DROPPED = AutonomousMode.PIXEL_DROPPED;

    public Pixel(AutonomousMode.PropPosition propPosition, RobotDrive drive,
                 NormalizedColorSensor colorSensor, Servo purplePixelServo,
                 DcMotor armAngleMotor, DcMotor armExtensionMotor, Servo wristServo,
                 Servo bucketServo) {
        this.teamPropPosition = propPosition;
        this.drive = drive;
        this.colorSensor = colorSensor;
        this.purplePixelServo = purplePixelServo;
        this.armAngleMotor = armAngleMotor;
        this.armExtensionMotor = armExtensionMotor;
        this.wristServo = wristServo;
        this.bucketServo = bucketServo;
    }

    public void deliverPayload(boolean goToBoard) {
        // we are already forward, so we just do this

        if (teamPropPosition == AutonomousMode.PropPosition.LEFT) {
            leftProtocol();
        } else if (teamPropPosition == AutonomousMode.PropPosition.CENTER) {
            centerProtocol();
        } else if (teamPropPosition == AutonomousMode.PropPosition.RIGHT) {
            rightProtocol();
        }

//        switch (teamPropPosition) {
//            case LEFT:
//                drive.turn(Turn.LEFT_90);
//                break;
//            case CENTER:
//                break;
//            case RIGHT:
//                drive.turn(Turn.RIGHT_90);
//                break;
//        }
//
//        if (once) return;
//
////        drive.drive(Drive.STARTRIGHT_CENTER_START);
//
//        long startTime = System.currentTimeMillis();
//        if (teamPropPosition == PropPosition.LEFT) {
//            drive.drive(Drive.BACKWARDS_SLOW);
//        } else {
//            drive.drive(Drive.FORWARDS_SLOW);
//        }
//
//        boolean detectedTape = false;
//        while (!detectedTape) {
//            /* Color sensor check START */
//            NormalizedRGBA colors = colorSensor.getNormalizedColors();
//            Color.colorToHSV(colors.toColor(), hsvValues);
//            /* Logic to what color tape it is over. */
//            float saturation = hsvValues[1];
//            detectedTape = (saturation >= 0.6) || (colors.red > 0.04);
//        }
//        drive.drive(Drive.STOP);
//        telemetry.addLine("Detected Pixel.");
//        long timeCrawled = System.currentTimeMillis() - startTime;
//        purplePixelServo.setPosition(PIXEL_DROPPED);
//
//        if (!goToBoard) return;
//
//        if (teamPropPosition != PropPosition.LEFT) {
//            drive.drive(Drive.BACKWARDS_SLOW);
//            drive.sleepMillis(timeCrawled);
//            drive.drive(Drive.STOP);
//            if (once) return;
//            if (teamPropPosition == PropPosition.RIGHT) {
////                drive.drive(Drive.STARTLEFT_CENTER_START);
//                drive.turn(Turn.GO_180);
//            } else if (teamPropPosition == PropPosition.CENTER) {
//                drive.turn(Turn.LEFT_90);
//            }
//            drive.drive(Drive.TO_BOARD);
//        } else {
//            drive.drive(Drive.BACKWARDS_SLOW);
//            drive.sleepMillis(drive.toBoardTime);
//            drive.drive(Drive.STOP);
//        }
    }

    private void leftProtocol() {
        // pixel drop only code:
        // go until tape
        long centerTime = goToTape(Direction.FORWARDS);
        // turn right
        drive.turn(Turn.LEFT_90);
        // go until tape
        long leftTime = goToTape(Direction.FORWARDS);
        // release pixel
        purplePixelServo.setPosition(PIXEL_DROPPED);
        // get away from the pixel
        drive.drive(Drive.FROM_BOARD_BACK);


        // code for dropping the yellow pixel as well:
        // go until center
        // turn right
        // go back until tape
        // release pixel
        // go back little bit
        // 180
        //go until tape
    }

    private void centerProtocol() {
        // go until tape
        long startTime = goToTape(Direction.FORWARDS);
        // release pixel
        purplePixelServo.setPosition(PIXEL_DROPPED);

       // pixel drop only code:
       drive.drive(Drive.FROM_BOARD_BACK);

       // code for dropping the yellow pixel as well:
//        drive.goForTime(startTime, Direction.BACKWARDS, Speed.SLOW);
//
//        drive.turn(Turn.LEFT_90);
//        // strafe into position
//        drive.drive(Drive.CENTER_TO_MIDDLE); // currently does nothing
//        // extend pixel arm
//        extendArm();
//        // drive to the board
//        drive.drive(Drive.CENTER_TO_BOARD);
//        // release the pixel from the arm
//        releasePixel();
//        // drive back a bit so we have room to retract the arm
//        drive.drive(Drive.FROM_BOARD_BACK);
//        // retract pixel arm
//        retractArm();
//        // go back to the board
//        drive.drive(Drive.FROM_BOARD_FORWARDS);
    }

    private void rightProtocol() {
        // go until tape
        long centerTime = goToTape(Direction.FORWARDS);
        // turn right
        drive.turn(Turn.RIGHT_90);
        // go until tape
        long rightTime = goToTape(Direction.FORWARDS);
        // release pixel
        purplePixelServo.setPosition(PIXEL_DROPPED);

        // pixel drop only code:
        // get away from the pixel
        drive.drive(Drive.FROM_BOARD_BACK);

        // code for dropping the yellow pixel as well:
    }

    private void extendArm() {
        double startExtendTime = System.currentTimeMillis();
        // Begin raising armAngleMotor
        armAngleMotor.setTargetPosition(DriveMode.raisedArmPosition);
        if ((System.currentTimeMillis() - startExtendTime) > DriveMode.wristArmChangeDelay) {
            wristServo.setPosition(DriveMode.wristArmExtendedPosition); // USE EXTENSION OF ARM MOTOR TO DETERMINE EXTENSION
        } // Separate if statement for separate tuning
        if ((System.currentTimeMillis() - startExtendTime) > DriveMode.armExtensionDelay) {
            armExtensionMotor.setTargetPosition(DriveMode.armAngleExtendedPosition);
        }
        drive.sleepMillis(1000);
    }
    // elements of typpographic style

    private void retractArm() {
        armAngleMotor.setTargetPosition(0); //retract pixel arm
        armExtensionMotor.setTargetPosition(0);
        wristServo.setPosition(DriveMode.wristArmRetractedPosition);

        drive.sleepMillis(1500);
    }

    private void releasePixel() {
        bucketServo.setPosition(DriveMode.dropBucketPixelPosition);
        drive.sleepMillis(1500);
    }

    public enum Direction {
        FORWARDS,
        BACKWARDS,
    }
    private long goToTape(Direction direction) {
        long startTime = System.currentTimeMillis();
        if (direction == Direction.FORWARDS) {
            drive.drive(Drive.FORWARDS_SLOW);
        } else {
            drive.drive(Drive.BACKWARDS_SLOW);
        }

        boolean detectedTape = false;
        while (!detectedTape) {
            /* Color sensor check START */
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            /* Logic to what color tape it is over. */
            float saturation = hsvValues[1];
            detectedTape = (saturation >= 0.6) || (colors.red > 0.04);
        }
        drive.drive(Drive.STOP);
//        telemetry.addLine("Detected Pixel.");
        long timeCrawled = System.currentTimeMillis() - startTime;
//        purplePixelServo.setPosition(PIXEL_DROPPED);

        return timeCrawled;
    }
}
