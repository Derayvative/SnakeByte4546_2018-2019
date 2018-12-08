package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;

//@Autonomous
public class TestTeamMarkerDrop extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        {
            initialize();
            waitForStart();
            setTeamMarker();
            pRightTurn(45);
            sleep(1000);
            setPower(-.2);
            sleep(4000);
            dropTeamMarker();
            sleep(1000);
            setTeamMarker();
            setZero();
        }
    }
}

