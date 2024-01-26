package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotDrive {

    public enum Turn {
        RIGHT_90,
        LEFT_90,
        GO_180,
    }

    public enum Drive {
        STARTLEFT_CENTER_START,
        STARTRIGHT_CENTER_START,
        FORWARD_SLOW,
        BACKWARDS_SLOW,
        STOP,
    }

    public static double motorSlowSpeed = 0.2;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private double startLeftCenterStartAmount = 300;
    private double startRightCenterStartAmount = 300;

    public RobotDrive(HardwareMap hardwareMap){
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_back");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
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

    public void turn90() {
        // do stuff
    }

    public void drive(Drive driveAmount) {
        double sleep_amount = 0;

        switch (driveAmount) {
            case STARTLEFT_CENTER_START:
                sleep_amount = startLeftCenterStartAmount;
                break;
            case STARTRIGHT_CENTER_START:
                sleep_amount = startRightCenterStartAmount;
                break;
        }

        motorsForward();
        sleepMillis(sleep_amount);
        motorsOff();
    }

    public void turn(Turn turnAmount) {
        switch (turnAmount) {
            case LEFT_90:
                motorsTurnLeft();
//                sleepMillis(TheUltimateQuestionToLifeTheUniverseAndEverything);
                motorsOff();
                break;
            case RIGHT_90:
                motorsTurnRight();
//                sleepMillis(asdasdiadijadjsadjks);
                motorsOff();
                break;
        }
    }

    private void motorsTurnLeft() {
        setMotorPowers(-1, 1, -1, 1);
    }

    private void motorsTurnRight() {
        setMotorPowers(1, -1, 1, -1);
    }

    private void motorsStrafeLeft() {
        setMotorPowers(1, -1, 1, -1);
    }

    private void motorsStrafeRight() {
        setMotorPowers(-1, 1, -1, 1);
    }

    private void motorsForward() {
        setAllMotorPowers(1);
    }

    void motorsForwardSlow() {
        setAllMotorPowers(motorSlowSpeed);
    }

    private void motorsReverse() {
        setAllMotorPowers(-1);
    }

    private void motorsReverseSlow() {
        setAllMotorPowers(-motorSlowSpeed);
    }

    private void motorsOff() {
        setAllMotorPowers(0);
    }

    private void sleepMillis(double time) {
        double start = System.currentTimeMillis();
        while (System.currentTimeMillis() < (start + time));
    }
}
