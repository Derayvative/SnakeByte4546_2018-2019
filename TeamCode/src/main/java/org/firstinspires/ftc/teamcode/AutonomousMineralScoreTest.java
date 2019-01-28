package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.GATE_DOWN_POSITION;

@Autonomous
public class AutonomousMineralScoreTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        middleIntake.setPower(-1);
        outerIntake.setPower(-0.25);
        PEncoderSetPowerBackward(1400);
        lowerTeamMarker();
        sleep(2000);
        liftToHeightAndMove(8200, 1200);

        setPower(0.25);
        sleep(1000);
        setPower(0);
        gateServo.setPosition(GATE_DOWN_POSITION);
        middleIntake.setPower(0);
        outerIntake.setPower(0);
        sleep(1000);
        PEncoderSetPowerBackward(300);
        liftToHeightZero(1000);
        turnToPosition(-90);

        maintainHeading(-90, -0.5, 6000);
    }
}
