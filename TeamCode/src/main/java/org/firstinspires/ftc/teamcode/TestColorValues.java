package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

//@Autonomous
public class TestColorValues extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        initialize();
        waitForStart();
        for (double i = 0; i <= 1.0; i += 0.1){
            sleep(300);
            basketServo.setPosition(i);
            gateServo.setPosition(i);
            telemetry.addData("I", i);
            telemetry.update();
        }
    }

}
