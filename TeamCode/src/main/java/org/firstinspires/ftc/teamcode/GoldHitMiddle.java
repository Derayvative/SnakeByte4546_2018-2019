package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class GoldHitMiddle extends AutoOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        moveTime(1500, .4);
        sleep(1000);
        moveToRange(15.0);
        setTeamMarker();
        sleep(1000);
        dropTeamMarker();
        sleep(1000);
        setTeamMarker();
        pRightTurn(55);
        pMoveBackward(5000);


    }
}
