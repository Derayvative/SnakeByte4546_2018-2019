package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous (name = "Howdy")
public class GoldHitMiddle extends AutoOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        powerLiftUpTime(-1, 400);
        powerLiftUpTime(1, 1250);



    }
}
