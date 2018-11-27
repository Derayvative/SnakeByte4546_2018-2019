package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous
public class CraterSideAddOn extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        initializeDriveTrainOnly();
        CS = hardwareMap.colorSensor.get("tapeSensor");
        DS = hardwareMap.get(DistanceSensor.class, "tapeSensor");
        waitForStart();
        setPower(-.1);
        while(opModeIsActive()){
            //if (senseTape(150)){
                setZero();
                break;
           // }
        }
        sleep(1000);
        setPower(.05);
        while(opModeIsActive()){
            //if (senseTape(150)){
                setZero();
                break;
            //}
        }
        turnToPositionPI(-90);
        pMoveForward(3000);
        sleep(1000);
        pLeftTurn(45);
        sleep(1000);
        moveToRangePIStraightenToStartAngle(20);
        sleep(1000);
        dropTeamMarker();
        sleep(2000);
        setTeamMarker();
        setPower(-1);
        while(opModeIsActive())
        {
            if (getGyroPitch() > 12){
                break;
            }
        }
        setZero();

    }
}
