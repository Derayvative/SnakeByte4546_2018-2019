package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous
public class MovementTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        // Gold - Left

        precisionTurnToPosition(-25);
        sleep(100);

        pMoveBackward(350);

        outerIntake.setPower(-1);
        middleIntake.setPower(-1);
        sleep(1000);

        pLeftTurn(10); // might be left
        setPower(-.4);


        // Gold - Middle

        pMoveBackward(350);

        outerIntake.setPower(-1);
        middleIntake.setPower(-1);
        sleep(1000);

        setPower(-.4);


        // Gold - Right

        precisionTurnToPosition(25);
        sleep(100);

        pMoveBackward(350);

        outerIntake.setPower(-1);
        middleIntake.setPower(-1);
        sleep(1000);

        pRightTurn(10); // might be right
        setPower(-.4);




    }
}