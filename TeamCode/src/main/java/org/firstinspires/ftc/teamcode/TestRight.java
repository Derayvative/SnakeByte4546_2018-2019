package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.GATE_DOWN_POSITION;

@Autonomous
public class TestRight extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        precisionTurnToPosition(30);
        middleIntake.setPower(-1);
        outerIntake.setPower(-0.25);
        PEncoderSetPowerBackward(1400);
        sleep(1000);
        liftToHeightAndMove(8200, 1250);
        setPower(0.25);
        sleep(1000);
        precisionTurnToPosition(0);
        gateServo.setPosition(GATE_DOWN_POSITION);
        sleep(2000);
        PEncoderSetPowerBackward(400);
        liftToHeight(0);
    }
}
