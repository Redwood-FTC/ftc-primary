/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 * <p>
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * <p>
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 * <p>
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 * <p>
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 * <p>
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Drive Mode", group = "Linear Opmode")
public class DriveMode extends OpMode {

    // Declare OpMode members for each of the 4 motors.
    protected final ElapsedTime runtime = new ElapsedTime();
    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;
    protected DcMotor winchMotor = null;
    protected DcMotor armAngleMotor = null;
    protected DcMotor armExtensionMotor = null;
    protected DcMotor intakeMotor = null;

    protected Servo hookAngleServo = null;
    protected Servo launchServo = null;
    protected Servo wristServo = null;
    protected Servo bucketServo = null;
    protected Servo intakeAngleServo = null;
    protected Servo hookReleaseServo = null;
    protected Servo hookWristServo = null;

    /* "Magic number" constants for physical positions. */
    protected final double stoppedMotorPower = 0.0;
    protected final double stoppedContinuousServoPosition = 0.5;
    protected final double hookArmHangingAngle = 0.17;
    protected final double hookArmDroneAngle = 0.7;
    protected final double unwindWinchPower = -1.0;
    protected final double windWinchPower = 1.0;
    protected final double raisedIntakePosition = 1.0;
    protected final double loweredIntakePosition = 0.0;
    protected final double releasedHookPosition = 0.0;
    protected final double heldHookPosition = 1.0;
    protected final double dropBucketPixelPosition = 1.0;
    protected final double pauseBucketPixelPosition = 0.5;
    protected final double loadBucketPixelPosition = 0.0;

    /* Settable positions/powers used in performActions() */
    private double hookAngleServoPosition = 0.0;
    private double winchMotorPower = 0.0;
    private double intakeAngleServoPosition = 0.0;
    private double hookReleaseServoPosition = 0.0;
    private double bucketServoPosition = 0.0;

    private double leftFrontDrivePower = 0.0;
    private double rightFrontDrivePower = 0.0;
    private double leftBackDrivePower = 0.0;
    private double rightBackDrivePower = 0.0;

    @Override
    public void init() {
        //double LAUNCH_SERVO_OPEN = 0.5;
        //double LAUNCH_SERVO_CLOSED = 0.05;

        // Motor Initialization START
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        armAngleMotor = hardwareMap.get(DcMotor.class, "arm_angle_motor");
        armAngleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armAngleMotor.setPower(1.0);
        armAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngleMotor.setTargetPosition(0);
        armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension_motor");
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtensionMotor.setPower(0.4); // Controlled automatically, fine tuning not needed
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setTargetPosition(0);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setPower(0);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setPower(0);

        winchMotor = hardwareMap.get(DcMotor.class, "winch_motor");
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Motor Initialization END

        // Servo Initialization START
        hookAngleServo = hardwareMap.get(Servo.class, "hook_angle_servo");
        hookAngleServo.setPosition(0.8);

        launchServo = hardwareMap.get(Servo.class, "launch_servo");
        launchServo.setPosition(0.85);

        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        wristServo.setPosition(1);

        bucketServo = hardwareMap.get(Servo.class, "bucket_servo");
        bucketServo.setPosition(0.5);

        intakeAngleServo = hardwareMap.get(Servo.class, "intake_angle_servo");
        intakeAngleServo.setPosition(1);

        hookWristServo = hardwareMap.get(Servo.class, "hook_wrist_servo");
        hookWristServo.setPosition(1.0); //was .25

        hookReleaseServo = hardwareMap.get(Servo.class, "hook_release_servo");
        hookReleaseServo.setPosition(1);
        // Servo Initialization END
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    protected long timeWristControlled = System.currentTimeMillis();
    protected boolean pixelDropMode = false;
    protected long timePixelModeChanged = System.currentTimeMillis();

    public class InputMapping {
        /**
         * Robot's forward-backward movement. Ranges from -1 to 1, inclusive.
         */
        public double axial = -gamepad1.left_stick_y; // Pushing stick forward gives negative value.

        /**
         * Robot's left-right movement. Ranges from -1 to 1, inclusive.
         */
        public double lateral = gamepad1.left_stick_x;

        /**
         * Robot's rotation movement. Ranges from -1 to 1, inclusive.
         */
        public double yaw = gamepad1.right_stick_x;

        /**
         * Rotate hook arm to hanging angle.
         */
        public boolean goToHangingAngle = gamepad2.dpad_up;

        /**
         * Rotate hook arm to drone launch angle.
         */
        public boolean goToDroneAngle = gamepad2.dpad_down;

        /**
         * Wind in hanging winch.
         */
        public boolean windWinch = gamepad2.dpad_right;
        /**
         * Unwind hanging winch.
         */
        public boolean unwindWinch = gamepad2.dpad_left;

        /**
         * Raise the intake off of the floor.
         */
        public boolean raiseIntake = gamepad2.a;

        /**
         * Release the hook.
         */
        public boolean releaseHook = gamepad2.b;

        /**
         * Drop pixels from the bucket.
         */
        public boolean dropBucketPixel = gamepad1.a;

        /**
         * Pause bucket wheel spinning
         */
        public boolean pauseBucket = gamepad1.b;

        /**
         * Move pixel arm to drop position.
         */
        public boolean goToDropPosition = gamepad1.right_trigger > 0.05;

        /**
         * Move pixel arm to load position.
         */
        public boolean goToLoadPosition = gamepad1.left_trigger > 0.05;

        /**
         * Lift the pixel arm.
         */
        public boolean liftPixelArm = gamepad1.y;


        /**
         * Lower the pixel arm.
         */
        public boolean lowerPixelArm = gamepad1.x;

        /**
         * Launch the drone.
         */
        public boolean launchDrone = gamepad2.right_bumper;
    }

    protected InputMapping getInput() {
        return new InputMapping();
    }

    @Override
    public void loop() {
        double maxDriveMotorPower;

        InputMapping input = getInput();

        // POV Mode uses left joystick to go forward (up/down) & strafe (left/right), and right
        // joystick to rotate (left/right.

        if (input.goToHangingAngle) {
            hookAngleServoPosition = hookArmHangingAngle;
        } else if (input.goToDroneAngle) {
            hookAngleServoPosition = hookArmDroneAngle;
        }

        if (input.unwindWinch) {
            winchMotorPower = unwindWinchPower;
        } else if (input.windWinch) {
            winchMotorPower = windWinchPower;
        } else {
            winchMotorPower = stoppedMotorPower;
        }

        // Test code for intake_angle_servo
        // Remember to find correct values later
        if (input.raiseIntake) {
            intakeAngleServoPosition = raisedIntakePosition;
        } else if (runtime.now(TimeUnit.MILLISECONDS) > 700) {
            intakeAngleServoPosition = loweredIntakePosition;

            // Test code for intake_angle_servo
            // Remember to find correct values later

            if (input.releaseHook) {
                hookReleaseServoPosition = releasedHookPosition;
            } else {
                hookReleaseServoPosition = heldHookPosition;
            }

            if (input.dropBucketPixel) {
                bucketServoPosition = dropBucketPixelPosition;
                intakeMotor.setPower(-1);
            } else if (input.pauseBucket) {
                bucketServoPosition = pauseBucketPixelPosition;
                intakeMotor.setPower(1);
            } else {
                bucketServoPosition = loadBucketPixelPosition;
                intakeMotor.setPower(1);
            }

            if (input.goToDropPosition) {
                pixelDropMode = true;
                timePixelModeChanged = System.currentTimeMillis();
            } else if (input.goToLoadPosition) {
                pixelDropMode = false;
                timePixelModeChanged = System.currentTimeMillis();
            }

            if (pixelDropMode) { // SET POWER FOR ALL 3
                // Begin raising armAngleMotor
                armAngleMotor.setTargetPosition(7200);
                if ((System.currentTimeMillis() - timePixelModeChanged) > 1000) {
                    wristServo.setPosition(0.76); // USE EXTENSION OF ARM MOTOR TO DETERMINE EXENSION
                } // Separate if to allow separate tuning
                if ((System.currentTimeMillis() - timePixelModeChanged) > 1000) {
                    armExtensionMotor.setTargetPosition(-2000); //was -2100
                }

                //put out armExtensionMotor after 200-ish mils
                //move wristServo same time
            } else if (!pixelDropMode) {
                armAngleMotor.setTargetPosition(0);
                armExtensionMotor.setTargetPosition(0);
                wristServo.setPosition(1.0);
            }

            if (input.lowerPixelArm) {
                armAngleMotor.setPower(-1.0);
            } else if (input.liftPixelArm) {
                armAngleMotor.setPower(1.0);
            } else {
                armAngleMotor.setPower(0.0);
            }


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontDrivePower = input.axial + input.lateral + input.yaw;
            rightFrontDrivePower = input.axial - input.lateral - input.yaw;
            leftBackDrivePower = input.axial - input.lateral + input.yaw;
            rightBackDrivePower = input.axial + input.lateral - input.yaw;

            // Normalize the values so no wheel power exceeds 100%.
            // This ensures that the robot maintains the desired motion.
            maxDriveMotorPower = Math.max(Math.abs(leftFrontDrivePower), Math.abs(rightFrontDrivePower));
            maxDriveMotorPower = Math.max(maxDriveMotorPower, Math.abs(leftBackDrivePower));
            maxDriveMotorPower = Math.max(maxDriveMotorPower, Math.abs(rightBackDrivePower));

            if (maxDriveMotorPower > 1.0) {
                leftFrontDrivePower /= maxDriveMotorPower;
                rightFrontDrivePower /= maxDriveMotorPower;
                leftBackDrivePower /= maxDriveMotorPower;
                rightBackDrivePower /= maxDriveMotorPower;
            }

            long planeLaunched = -1;
            if (input.launchDrone) {
                    if (planeLaunched == -1) {
                    launchServo.setPosition(1);
                    planeLaunched = System.currentTimeMillis();
                } //after plane is launched, same button moves the servo back
            }
            if ((System.currentTimeMillis() - planeLaunched) >= 500) {
                launchServo.setPosition(0.85);
            }

            performActions();
            sendTelemetry();
        }
    }


    /**
     * Execute motor/servo outputs.
     */
    protected void performActions() {
        hookAngleServo.setPosition(hookAngleServoPosition);
        winchMotor.setPower(winchMotorPower);
        intakeAngleServo.setPosition(intakeAngleServoPosition);
        hookReleaseServo.setPosition(hookReleaseServoPosition);
        bucketServo.setPosition(bucketServoPosition);
        // Drive motors
        leftFrontDrive.setPower(leftFrontDrivePower);
        rightFrontDrive.setPower(rightFrontDrivePower);
        leftBackDrive.setPower(leftBackDrivePower);
        rightBackDrive.setPower(rightBackDrivePower);
    }

    /**
     * Send telemetry about robot state back to Driver Station.
     */
    protected void sendTelemetry() {
        telemetry.addData("Extension_Motor encoder value: ", armExtensionMotor.getCurrentPosition());
        telemetry.addData("Angle_Motor encoder value: ", armAngleMotor.getCurrentPosition());
        telemetry.addData("Angle_Intake_Servo encoder value: ", intakeAngleServo.getPosition());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontDrive.getPower(), rightFrontDrive.getPower());
        telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
        telemetry.update();
    }
}
