
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name = "DistanceSensorT", group = "TeleOp")
    public class DistanceSensorT extends LinearOpMode {
        private Rev2mDistanceSensor distanceSensor;

        @Override
        public void runOpMode() {
            distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");

            waitForStart();

            while (opModeIsActive()) {
                double distanceInInches = distanceSensor.getDistance(DistanceUnit.INCH);

                if (distanceInInches < 12) {
                    telemetry.addData("Distance", "Backboard approaching");
                    telemetry.update();


                }



                idle();
            }
        }
    }



