/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Position Setter", group="Linear Opmode")
public class PositionSetter extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor extensionMotor = null;
    private DcMotor armAngleMotor = null;
    private DcMotor winchMotor = null;
    private DcMotor armExtensionMotor = null;
    private Servo hookReleaseServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        extensionMotor = hardwareMap.get(DcMotor.class, "arm_extension_motor");

        armAngleMotor = hardwareMap.get(DcMotor.class, "arm_angle_motor");
        armAngleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armAngleMotor.setPower(0);
        armAngleMotor.setTargetPosition(0);

        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension_motor");
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armExtensionMotor.setPower(0);
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setTargetPosition(0);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        winchMotor = hardwareMap.get(DcMotor.class, "winch_motor");
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hookReleaseServo = hardwareMap.get(Servo.class, "hook_release_servo");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        extensionMotor.setDirection(DcMotor.Direction.REVERSE);

        double hookReleasePosition = 0.5;
        double hookReleaseSpeed = 0.01;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.x) {
                armAngleMotor.setPower(-1.0);
            } else if (gamepad1.y) {
                armAngleMotor.setPower(1.0);
            } else {
                armAngleMotor.setPower(0.0);
            }

            double winchPower;
            if (gamepad2.dpad_left) {
                winchPower = -1.0;
            } else if (gamepad2.dpad_right) {
                winchPower = 1.0;
            } else {
                winchPower = 0.0;
            }
            winchMotor.setPower(winchPower);

            if (gamepad1.right_trigger > 0.05) {
                armExtensionMotor.setTargetPosition(-2100);
                armExtensionMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.05) {
                armExtensionMotor.setPower(gamepad1.left_trigger);
                armExtensionMotor.setTargetPosition(0);
            } else {
                armExtensionMotor.setPower(0);
            }

            if (gamepad2.left_bumper) {
                hookReleasePosition += hookReleaseSpeed;
            } else if (gamepad2.right_bumper){
                hookReleasePosition -= hookReleaseSpeed;
            }
            hookReleaseServo.setPosition(hookReleasePosition);

            telemetry.addData("hookReleasePosition", hookReleaseServo.getPosition());

           telemetry.addData("amount_tilted", armAngleMotor.getCurrentPosition());
           telemetry.addData("amount_arm_extended", armExtensionMotor.getCurrentPosition());
           telemetry.addData("amount_extended", extensionMotor.getCurrentPosition());
           telemetry.addData("Winch_Motor_encoder value", winchMotor.getCurrentPosition());
           telemetry.update();
        }
    }
}
