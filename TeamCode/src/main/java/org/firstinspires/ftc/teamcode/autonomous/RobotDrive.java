package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    }

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public static double motorSlowSpeed = 0.1;

    // drive constants
    public static double startLeftCenterStartAmount = 400;
    public static double startRightCenterStartAmount = startLeftCenterStartAmount;
    public static double toPixelCenter = 1200;

    public static double turnLeft90Amount = 985;
    public static double turnRight90Amount = 985;

    public RobotDrive(HardwareMap hardwareMap){
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotorEx.class, "left_back");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.get(DcMotorEx.class, "right_back");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
    }

    private void setMotorPowers(double leftFrontPower, double leftRearPower, double rightRearPower, double rightFrontPower) {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    private void setAllMotorPowers(double power) {
        setMotorPowers(power, power, power, power);
    }

    public void drive(Drive driveAmount) {
        double sleep_amount = 0;

        switch (driveAmount) {
            case STARTLEFT_CENTER_START:
                motorsStrafeRight();
                sleepMillis(startLeftCenterStartAmount);
                motorsOff();
                return;
            case STARTRIGHT_CENTER_START:
                motorsStrafeLeft();
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

    private void motorsTurnLeft() {
        setMotorPowers(-0.5, -0.5, 0.5, 0.5);
    }

    private void motorsTurnRight() {
        setMotorPowers(0.5, 0.5, -0.5, -0.5);
    }

    private void motorsStrafeLeft() {
        setMotorPowers(-0.5, 0.5, -0.5, 0.5);
    }

    private void motorsStrafeRight() {
        setMotorPowers(0.5, -0.5, 0.5, -0.5);
    }

    private void motorsForward() {
        setAllMotorPowers(0.5);
    }

    void motorsForwardSlow() {
        setAllMotorPowers(motorSlowSpeed);
    }

    private void motorsReverse() {
        setAllMotorPowers(-0.5);
    }

    private void motorsReverseSlow() {
        setAllMotorPowers(-motorSlowSpeed);
    }

    private void motorsOff() {
        setAllMotorPowers(0);
    }

    public void sleepMillis(double time) {
        double start = System.currentTimeMillis();
        while (System.currentTimeMillis() < (start + time));
    }
}
