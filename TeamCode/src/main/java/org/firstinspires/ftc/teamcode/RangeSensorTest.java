package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RangeSensorTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        moveToRangeBasic(15);
        moveToRangeBasic(50);
        moveToRangeBasic(15);
        moveToRangeBasic(70);
    }
}
