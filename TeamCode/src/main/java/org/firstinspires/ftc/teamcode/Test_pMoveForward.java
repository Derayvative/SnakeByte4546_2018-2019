package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class Test_pMoveForward extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        initialize();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("FL",FL.getCurrentPosition());
            telemetry.addData("FL",FR.getCurrentPosition());
            telemetry.addData("FL",BL.getCurrentPosition());
            telemetry.addData("FL",BR.getCurrentPosition());
            telemetry.update();
        }
    }
}