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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.fenum.qual.SwingCompassDirection;

import java.util.concurrent.TimeUnit;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear Opmode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo hoistServo = null;
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

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

        hoistServo = hardwareMap.get(Servo.class, "hoist_servo");
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

        armAngleMotor = hardwareMap.get(DcMotor.class, "arm_angle_motor");
        armAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension_motor");
        armExtensionMotor.setPower(0);
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setTargetPosition(0);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setPower(0);

        hookReleaseServo = hardwareMap.get(Servo.class, "hook_release_servo");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        double wristPosition = 1;
        long timeWristControlled = System.currentTimeMillis();
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double hoistPower;
            if (gamepad2.dpad_up) {
                hoistServo.setPosition(0.0);
            } else if (gamepad2.dpad_down) {
                hoistServo.setPosition(1.0);
            } else {
                //hoistPower = 0.5;
            }
            //hoistServo.setPosition(hoistPower);

//            double winchPower;
//            if (gamepad1.dpad_left) {
//                winchPower = -1.0;
//            } else if (gamepad1.dpad_right) {
//                winchPower = 1.0;
//            } else {
//                winchPower = 0.0;
//            } IMPORTANT: UNCOMMENT LATER (commented for testing intake_angle_servo)

            // Test code for intake_angle_servo
            // Remember to find correct values later


            if (runtime.now(TimeUnit.MILLISECONDS) > 700) {
                intakeAngleServo.setPosition(0);
            }

            if (gamepad1.left_bumper && ((System.currentTimeMillis() - timeWristControlled) > 1000)) {
                //takes 3 seconds to go between positions
                //delay may need to be extended to prevent malfunction
                //TODO: ADD PROTECTION THAT ARM MUST BE EXTENDED
                wristPosition = (wristPosition == 1 ? 0 : 1);
                timeWristControlled = System.currentTimeMillis();
            }
            wristServo.setPosition(wristPosition);

            if (gamepad1.a) {
                bucketServo.setPosition(1);
                intakeMotor.setPower(1);
            } else if (gamepad1.b) {
                bucketServo.setPosition(0);
                intakeMotor.setPower(0);
            } else {
                bucketServo.setPosition(0.5);
                intakeMotor.setPower(0);
            }

            if (gamepad1.right_trigger > 0.05) {
                armExtensionMotor.setTargetPosition(-2100);
                armExtensionMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.05) {
                armExtensionMotor.setPower(gamepad1.left_trigger);
                armExtensionMotor.setTargetPosition(0);
            } else {
                armExtensionMotor.setPower(0);
            }

            if (gamepad1.x) {
                armAngleMotor.setPower(-1.0);
            } else if (gamepad1.y) {
                armAngleMotor.setPower(1.0);
            } else {
                armAngleMotor.setPower(0.0);
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
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

//            long planeLaunched = -1;
//            if (gamepad1.right_bumper){
//                if (planeLaunched == -1) {
//                    launchServo.setPosition(LAUNCH_SERVO_OPEN);
//                    planeLaunched = System.currentTimeMillis();
//                } //after plane is launched, same button moves the servo back
//            }
//            if ((System.currentTimeMillis() - planeLaunched) >= 500) {
//                launchServo.setPosition(LAUNCH_SERVO_CLOSED);
//            }
//
//            winchMotor.setPower(winchPower); // UNCOMMENT LATER (commented for testing intake_angle_servo)

            telemetry.addData("Extension_Motor encoder value: ", armExtensionMotor.getCurrentPosition());
            telemetry.addData("Angle_Motor encoder value: ",  armAngleMotor.getCurrentPosition());
            telemetry.addData("Angle_Intake_Servo encoder value: ", intakeAngleServo.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
            //check if opmode not active, then wait while we close the intake angle servo
        }
    }
}
