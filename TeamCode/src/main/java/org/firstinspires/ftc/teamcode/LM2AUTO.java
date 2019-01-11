package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//
//
@Autonomous
public class LM2AUTO extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        powerLiftUpP(3500);
        powerLiftDownP(3500);

    }
}
