package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED BACKSTAGE Autonomous Mode")
public class RedBackAutonomousMode extends AutonomousMode{
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected StartingPosition getStartingPosition() {
        return StartingPosition.BACKSTAGE;
    }
}
