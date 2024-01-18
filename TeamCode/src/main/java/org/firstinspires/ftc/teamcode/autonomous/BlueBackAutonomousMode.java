package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE BACKSTAGE Autonomous Mode")
public class BlueBackAutonomousMode extends AutonomousMode{
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected StartingPosition getStartingPosition() {
        return StartingPosition.BACKSTAGE;
    }
}
