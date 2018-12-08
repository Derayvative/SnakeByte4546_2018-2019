package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous
public class TurnTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //setTeamMarker();
        waitForStart();
        turnToPosition(0);

    }
}
