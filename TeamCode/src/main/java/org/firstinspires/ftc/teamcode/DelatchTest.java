package org.firstinspires.ftc.teamcode;

public class DelatchTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        powerLiftDownP(200);
        powerLiftUpP(200);

    }
}
