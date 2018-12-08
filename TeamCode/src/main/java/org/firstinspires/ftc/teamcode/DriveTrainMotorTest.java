package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous
public class DriveTrainMotorTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        FL.setPower(0.4);
        telemetry.addData("FL","FL");
        telemetry.update();
        sleep(3000);
        FL.setPower(0);
        FR.setPower(0.4);
        telemetry.addData("FR","FR");
        telemetry.update();
        sleep(3000);
        FR.setPower(0);
        BL.setPower(0.4);
        telemetry.addData("BL","BL");
        telemetry.update();
        sleep(3000);
        BL.setPower(0);
        BR.setPower(0.4);
        telemetry.addData("BR","BR");
        telemetry.update();
        sleep(3000);
        BR.setPower(0);
    }
}
