package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleop.Pixel;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class RobotDrive {

    public static enum Turn {
        RIGHT_90,
        LEFT_90,
        GO_180,
    }

    public static enum Drive {
        STARTLEFT_CENTER_START,
        STARTRIGHT_CENTER_START,
        TO_PIXEL_CENTER,
        FORWARDS_SLOW,
        BACKWARDS_SLOW,
        STOP,
        LITTLE_BIT_LEFT,
        LITTLE_BIT_RIGHT,
        TO_BOARD,
        FROM_BOARD_FORWARDS,
        FROM_BOARD_BACK,
        LEFT_TO_MIDDLE,
        LEFT_TO_BOARD,
        CENTER_TO_MIDDLE,
        CENTER_TO_BOARD,
        RIGHT_TO_MIDDLE,
        RIGHT_TO_BOARD,
    }

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    // drive constants
    public static double startLeftCenterStartAmount = 300;
    public static double startRightCenterStartAmount = startLeftCenterStartAmount;
    public static double toPixelCenter = 900;
    public static double toBoardTime = 150;
    public static double universalMotorSpeed = 1280;
    public static double motorSlowSpeed = universalMotorSpeed * 0.5;
    public static double toBoardAmount = 1000;
    public static double fromBoardForwards = 0;
    public static double fromBoardBack = 0;
    public static double leftToMiddle = 0;
    public static double left = 0;
    public static double centerToMiddle = 0;
    public static double center = 0;
    public static double rightToMiddle = 0;
    public static double right = 0;

    public static double turnRight90Amount = 890;
    public static double turnLeft90Amount = turnRight90Amount; // separate because there have been consistency issues

    public RobotDrive(HardwareMap hardwareMap){
       leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
       leftRear = hardwareMap.get(DcMotorEx.class, "left_back");
       rightRear = hardwareMap.get(DcMotorEx.class, "right_back");
       rightFront = hardwareMap.get(DcMotorEx.class, "right_front");

        DcMotorEx[] motors = { leftFront, leftRear, rightRear, rightFront };

       for (DcMotorEx motor : motors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setVelocity(0);
//            motor.setMotorDisable();
//            motor.setPower(0.5);
//            motor.setMotorEnable();
        }

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

    }

    public void init() {
        leftFront.setPower(0.5);
        leftRear.setPower(0.5);
        rightFront.setPower(0.5);
        rightFront.setPower(0.5);
    }

    private void setMotorVelocities(double leftFrontPower, double leftRearPower, double rightRearPower, double rightFrontPower) {
        leftFront.setVelocity(leftFrontPower);
        rightFront.setVelocity(rightFrontPower);
        leftRear.setVelocity(leftRearPower);
        rightRear.setVelocity(rightRearPower);
    }

    private void setAllMotorVelocities(double velocity) {
        setMotorVelocities(velocity, velocity, velocity, velocity);
    }

    public void drive(Drive driveAmount) {
        double sleep_amount = 0;

        switch (driveAmount) {
            case STARTLEFT_CENTER_START:
//                motorsForward();
//                sleepMillis(1234123871);
                motorsStrafeRight();
                sleepMillis(startLeftCenterStartAmount);
                motorsOff();
                return;
            case STARTRIGHT_CENTER_START:
                motorsStrafeLeft();
//                sleepMillis(startRightCenterStartAmount);
                sleepMillis(startRightCenterStartAmount);
                motorsOff();
                return;
            case TO_PIXEL_CENTER:
                sleep_amount = toPixelCenter;
                break;
            case FORWARDS_SLOW:
                motorsForwardSlow();
                return;
            case BACKWARDS_SLOW:
                motorsReverseSlow();
                return;
            case STOP:
                motorsOff();
                return;
            case LITTLE_BIT_LEFT:
                motorsStrafeLeft();
                sleepMillis(startLeftCenterStartAmount);
                motorsOff();
                return;
            case LITTLE_BIT_RIGHT:
                motorsStrafeRight();
                sleepMillis(startLeftCenterStartAmount);
                motorsOff();
                return;
            case TO_BOARD:
                motorsReverse();
                sleepMillis(toBoardAmount);
                motorsOff();
                return;
        }

        motorsForward();
        sleepMillis(sleep_amount);
        motorsOff();
    }

    public void turn(Turn turnAmount) {
        switch (turnAmount) {
            case LEFT_90:
                motorsTurnLeft();
                sleepMillis(turnLeft90Amount);
                motorsOff();
                break;
            case RIGHT_90:
                motorsTurnRight();
                sleepMillis(turnRight90Amount);
                motorsOff();
                break;
            case GO_180:
                motorsTurnRight();
                sleepMillis(2 * turnLeft90Amount);
                motorsOff();
                break;
        }
    }

    public enum Speed {
        NORMAL,
        SLOW,
    }

    public void goForTime(long time, Pixel.Direction direction, Speed speed) {
        if (direction == Pixel.Direction.FORWARDS) {
            if (speed == Speed.NORMAL) {
                motorsForward();
            } else {
                motorsForwardSlow();
            }
        } else {
            if (speed == Speed.NORMAL) {
                motorsReverse();
            } else {
                motorsReverseSlow();
            }
        }
        sleepMillis(time);
        motorsOff();
    }

    private void motorsTurnLeft() {
        setMotorVelocities(-universalMotorSpeed, -universalMotorSpeed, universalMotorSpeed, universalMotorSpeed);
    }

    private void motorsTurnRight() {
        setMotorVelocities(universalMotorSpeed, universalMotorSpeed, -universalMotorSpeed, -universalMotorSpeed);
    }

    private void motorsStrafeLeft() {
        setMotorVelocities(-universalMotorSpeed, universalMotorSpeed, -universalMotorSpeed, universalMotorSpeed);
    }

    private void motorsStrafeRight() {
        setMotorVelocities(universalMotorSpeed, -universalMotorSpeed, universalMotorSpeed, -universalMotorSpeed);
    }

    private void motorsForward() {
        setAllMotorVelocities(universalMotorSpeed);
    }

    void motorsForwardSlow() {
        setAllMotorVelocities(motorSlowSpeed);
    }

    private void motorsReverse() {
        setAllMotorVelocities(-universalMotorSpeed);
    }

    private void motorsReverseSlow() {
        setAllMotorVelocities(-motorSlowSpeed);
    }

    private void motorsOff() {
        setAllMotorVelocities(0);
    }

    public void sleepMillis(double time) {
        double start = System.currentTimeMillis();
        while (System.currentTimeMillis() < (start + time));
    }
}
