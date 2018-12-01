package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class RangeSensorTrial extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Range", getRangeReading());
            telemetry.addData("Encoder", getAvgEncoder());
            telemetry.update();
            sleep(100);
        }
    }
}
