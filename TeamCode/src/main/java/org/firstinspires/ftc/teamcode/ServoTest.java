package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ServoTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        hangServo.setPosition(0.5);
        waitForStart();
        double timeOfPress = 0;
        while (opModeIsActive()){
            if (gamepad1.dpad_up && System.currentTimeMillis() - timeOfPress > 250){

                timeOfPress = System.currentTimeMillis();
                double pos = hangServo.getPosition()+0.05;
                hangServo.setPosition(pos);
            }
            if (gamepad1.dpad_down && System.currentTimeMillis() - timeOfPress > 250){

                timeOfPress = System.currentTimeMillis();
                double pos = hangServo.getPosition()-0.05;
                hangServo.setPosition(pos);
            }
            telemetry.addData("Servo", hangServo.getPosition());
            telemetry.update();
        }
    }
}
