package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

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
    private final float[] hsvValues = new float[3];
    private double PIXEL_DROPPED = AutonomousMode.PIXEL_DROPPED;

    public Pixel(AutonomousMode.PropPosition propPosition, RobotDrive drive,
                 NormalizedColorSensor colorSensor, Servo purplePixelServo) {
        this.teamPropPosition = propPosition;
        this.drive = drive;
        this.colorSensor = colorSensor;
        this.purplePixelServo = purplePixelServo;
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
        // go back

    }

    private void rightProtocol() {
        // go until tape
        long centerTime = goToTape(Direction.FORWARDS);
        // turn right
        drive.turn(Turn.RIGHT_90);
        // go until tape?
        long rightTime = goToTape(Direction.FORWARDS);
        // release pixel
        purplePixelServo.setPosition(PIXEL_DROPPED);
        // 
    }

    public enum Direction {
        FORWARDS,
        BACKWARDS,
    }
    public long goToTape(Direction direction) {
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
