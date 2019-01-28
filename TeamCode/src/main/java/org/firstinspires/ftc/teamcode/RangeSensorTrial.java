package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class RangeSensorTrial extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        precisionTurnToPosition(-32.5);
        PEncoderSetPowerBackward(1200);
        moveToRangeBasic(21, -32.5);
        precisionTurnToPosition(45);
        moveToRangeBasic(43, 45);
        lowerTeamMarker();
        precisionTurnToPosition(70);
        setPower(0.5);
        sleep(1500);
        maintainHeading(45, 0.5, 7000);
        /*
        while (opModeIsActive()){
            telemetry.addData("Range", getRangeReading());
            telemetry.addData("Encoder", getAvgEncoder());
            telemetry.update();
            sleep(100);
        }
        */
    }
}
