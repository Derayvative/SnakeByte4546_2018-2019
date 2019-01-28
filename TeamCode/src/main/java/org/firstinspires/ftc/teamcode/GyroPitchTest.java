package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous
public class GyroPitchTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        precisionTurnToPosition(30);
        PEncoderSetPowerBackward(1200);
        moveToRangeBasic(12, 30);
        precisionTurnToPosition(-45);
        moveToRangeBasic(13, -45);
        lowerTeamMarker();
        precisionTurnToPosition(63);
        setPower(0.5);
        sleep(1500);
        maintainHeading(45, 0.5, 7000);

    }
}
