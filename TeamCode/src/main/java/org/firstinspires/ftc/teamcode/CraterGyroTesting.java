package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//@Autonomous
public class CraterGyroTesting extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        double power = 0;


        while (opModeIsActive()) {
            telemetry.addData("Gyro Pitch", getGyroPitch());
            telemetry.addData("power", power);
            telemetry.update();

            // 16 degrees is max



            if (getGyroPitch() > -1) {
                power = .35;
                setPower(power);
            }

            else if (getGyroPitch() > -5) {
                power =  (.225/(Math.abs(getGyroPitch())));
                setPower(power);
            }

            else {
                telemetry.addData("threshold", "motionFinished");
                telemetry.update();
                break;
            }



        }

        setZero();
        sleep(3000);




    }
}