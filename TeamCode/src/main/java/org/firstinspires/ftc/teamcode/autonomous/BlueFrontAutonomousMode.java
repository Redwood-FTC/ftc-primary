package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE FRONTSTAGE Autonomous Mode")
public class BlueFrontAutonomousMode extends AutonomousMode{
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected StartingPosition getStartingPosition() {
        return StartingPosition.FRONTSTAGE;
    }
}
