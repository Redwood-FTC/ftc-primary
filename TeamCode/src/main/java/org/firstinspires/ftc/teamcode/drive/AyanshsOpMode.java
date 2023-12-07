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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Ayansh's OpMode", group="Linear Opmode")
public class AyanshsOpMode extends LinearOpMode {

/*
    This OpMode was made for Ayansh because he wanted his own OpMode for his own control scheme.
*/

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo hookAngleServo = null;
    private Servo launchServo = null;
    private DcMotor winchMotor = null;
    private Servo wristServo = null;
    private Servo bucketServo = null;
    private Servo intakeAngleServo = null;
    private DcMotor armAngleMotor = null;
    private DcMotor armExtensionMotor = null;
    private DcMotor intakeMotor = null;
    private Servo hookReleaseServo = null;

    @Override
    public void runOpMode() {
        double LAUNCH_SERVO_OPEN = 0.5;
        double LAUNCH_SERVO_CLOSED = 0.05;

        /* Motor Initialization START */
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension_motor");
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtensionMotor.setPower(0);
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setTargetPosition(0);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armAngleMotor = hardwareMap.get(DcMotor.class, "arm_angle_motor");
        armAngleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armAngleMotor.setPower(1);
        armAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngleMotor.setTargetPosition(0);
        armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension_motor");
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtensionMotor.setPower(0.4); //controlled automatically, fine tuning not needed
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setTargetPosition(0);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setPower(0);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setPower(0);
        /* Motor Initialization END */
        /* Servo Initialization START */
        hookAngleServo = hardwareMap.get(Servo.class, "hook_angle_servo");
        hookAngleServo.setPosition(0.2);

        launchServo = hardwareMap.get(Servo.class, "launch_servo");
        launchServo.setPosition(0.05);

        winchMotor = hardwareMap.get(DcMotor.class, "winch_motor");
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        wristServo.setPosition(1);

        bucketServo = hardwareMap.get(Servo.class, "bucket_servo");
        bucketServo.setPosition(0.5);

        intakeAngleServo = hardwareMap.get(Servo.class, "intake_angle_servo");
        intakeAngleServo.setPosition(1);

        hookReleaseServo = hardwareMap.get(Servo.class, "hook_release_servo");
        hookReleaseServo.setPosition(1);
        /* Servo Initialization END */


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        long timeWristControlled = System.currentTimeMillis();
        double wristPosition = 1;
        boolean pixelDropMode = false;
        long timePixelModeChanged = System.currentTimeMillis();
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double hookAnglePower;
            if (gamepad2.dpad_up) {
                hookAngleServo.setPosition(1.0); //go to hook (TEMP VALUE)
            } else if (gamepad2.dpad_down) {
                hookAngleServo.setPosition(0.7); //go to plane (TEMP VALUE)
            } else if (gamepad2.y) {
                hookAngleServo.setPosition(0.0);
            }
            //hookAngleServo.setPosition(hookAnglePower);

            double winchPower;
            if (gamepad2.dpad_left) {
                winchPower = -1.0;
            } else if (gamepad2.dpad_right) {
                winchPower = 1.0;
            } else {
                winchPower = 0.0;
            }
            winchMotor.setPower(winchPower);

            // Test code for intake_angle_servo
            // Remember to find correct values later
            if (gamepad2.a) {
                intakeAngleServo.setPosition(1);
            } else if (runtime.now(TimeUnit.MILLISECONDS) > 700) {
                intakeAngleServo.setPosition(0);

                // Test code for intake_angle_servo
                // Remember to find correct values later

                if (gamepad2.b) {
                    hookReleaseServo.setPosition(0);
                }

                if (gamepad1.dpad_right && (System.currentTimeMillis() - timeWristControlled > 200)) {
//                if (wristPosition < 1) {
//                    wristPosition += 0.1;
//                }
                    wristServo.setPosition(1);
                } else if (gamepad1.dpad_left && (System.currentTimeMillis() - timeWristControlled > 200)) {
//                if (wristPosition > 0) {
//                    wristPosition -= 0.1;
//                }
                    wristServo.setPosition(0.76); //WAS 0.8
                } else {
                    //wristServo.setPosition(1);
                    //theoretically nothing should happen
                }
                //wristServo.setPosition(wristPosition);

                //go to 1 when left trigger pressed?
//            if (gamepad1.left_bumper && ((System.currentTimeMillis() - timeWristControlled) > 1000)) {
//                //takes 3 seconds to go between positions
//                //delay may need to be extended to prevent malfunction
//                //TODO: ADD PROTECTION THAT ARM MUST BE EXTENDED
//                wristPosition = (wristPosition == 1 ? 0 : 1);
//                timeWristControlled = System.currentTimeMillis();
//            }

                if (gamepad1.a) {
                    bucketServo.setPosition(1);
                    intakeMotor.setPower(-1);
                } else if (gamepad1.b) {
                    bucketServo.setPosition(0);
                    intakeMotor.setPower(1);
                } else {
                    bucketServo.setPosition(1);
                    intakeMotor.setPower(1); //TEMPORARY TO STOP LOUD NOISE
                    //intakeMotor.setPower(1);
                }

//            if (gamepad1.right_trigger > 0.05) {
//                armExtensionMotor.setTargetPosition(-2100);
//                armExtensionMotor.setPower(gamepad1.right_trigger);
//            } else if (gamepad1.left_trigger > 0.05) {
//                armExtensionMotor.setPower(gamepad1.left_trigger);
//                armExtensionMotor.setTargetPosition(0);
//            } else {
//                armExtensionMotor.setPower(0);
//            }

                if (gamepad1.dpad_up) {
                    pixelDropMode = true;
                    timePixelModeChanged = System.currentTimeMillis();
                    //time set
                } else if (gamepad1.dpad_down) {
                    pixelDropMode = false;
                    timePixelModeChanged = System.currentTimeMillis();
                    //input mode
                    //same in reverse
                }

                if (pixelDropMode) { //SET POWER FOR ALL 3
                    //begin raising armanglemotor
                    armAngleMotor.setTargetPosition(7200);
                    if ((System.currentTimeMillis() - timePixelModeChanged) > 1000) {
                        wristServo.setPosition(0.76);
                    } //seperate if to allow separate tuning
                    if ((System.currentTimeMillis() - timePixelModeChanged) > 1000) {
                        armExtensionMotor.setTargetPosition(-2000); //was -2100
                    }

                    //put out armextensionmotor after 200-ish mils
                    //move wristservo same time
                } else if (!pixelDropMode) {
                    armAngleMotor.setTargetPosition(0);
                    armExtensionMotor.setTargetPosition(0);
                    wristServo.setPosition(1.0);
                }

//            if (gamepad1.dpad_up) {
//                armAngleMotor.setTargetPosition(7200); //was 7400
//            } else if (gamepad1.dpad_down) {
//                armAngleMotor.setTargetPosition(0);
//            } else {
//
//            }

                if (gamepad1.x) {
                    //use extension test to reset position
                    //FIXXDX
                    //armAngleMotor.setPower(-1.0);
                } else if (gamepad1.y) {
                    //armAngleMotor.setPower(1.0);
                } else {
                    //armAngleMotor.setPower(0.0);
                }


                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

//            long planeLaunched = -1;
//            if (gamepad2.left_trigger){
//                if (planefed == -1) {
//                    launchServo.setPosition(LAUNCH_SERVO_OPEN);
//                    planeLaunched = System.currentTimeMillis();
//                } //after plane is launched, same button moves the servo back
//            }
//            if ((System.currentTimeMillis() - planeLaunched) >= 500) {
//                launchServo.setPosition(LAUNCH_SERVO_CLOSED);
//            }
//

                telemetry.addData("Extension_Motor encoder value: ", armExtensionMotor.getCurrentPosition());
                telemetry.addData("Angle_Motor encoder value: ",  armAngleMotor.getCurrentPosition());
                telemetry.addData("Angle_Intake_Servo encoder value: ", intakeAngleServo.getPosition());
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();
                //check if opmode not active, then wait while we close the intake angle servo
            }
        }
    }
} //this threw an error without this
