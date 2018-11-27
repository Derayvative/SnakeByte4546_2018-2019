package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous
public class pturnAutoTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        //GoldDetectorColorSensor GD = new GoldDetectorColorSensor(CS, DS);
        waitForStart();
        /*

        */
        precisionTurnToPosition(-150);
        precisionTurnToPosition(-180);
        precisionTurnToPosition(150);

    }
}
