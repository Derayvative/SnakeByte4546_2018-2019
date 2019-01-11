package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.HANG_SERVO_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.HANG_SERVO_UP_POSITION;

@Autonomous
public class DelatchTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        hangServo.setPosition(HANG_SERVO_UP_POSITION);
        waitForStart();
        hangServo.setPosition(HANG_SERVO_DOWN_POSITION);
        sleep(2000);
        powerLiftUpP(1400);



    }
}
