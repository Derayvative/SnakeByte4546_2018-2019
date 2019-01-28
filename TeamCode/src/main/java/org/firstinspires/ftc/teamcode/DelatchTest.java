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
        //hangServo.setPosition(HANG_SERVO_UP_POSITION);
        waitForStart();
        hangServo.setPosition(HANG_SERVO_DOWN_POSITION);
        sleep(1500);
        moveBackwardEncoderSingle(0.2, 27);
        setPower(-0.2);
        sleep(500);
        powerLiftUpP(2000);
        setPower(0);
        PEncoderSetPowerBackward(15);
        powerLiftDownP(7000);
        powerLiftUpP(900);


    }
}
