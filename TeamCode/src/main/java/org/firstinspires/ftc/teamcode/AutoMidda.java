package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.GATE_DOWN_POSITION;

@Autonomous
public class AutoMidda extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        middleIntake.setPower(1);
        outerIntake.setPower(1);
        PEncoderSetPowerBackward(2600);
        lowerTeamMarker();
        precisionTurnToPosition(60);
        setPower(0.5);
        sleep(2000);
        maintainHeading(45, 0.5, 6000);
    }
}
