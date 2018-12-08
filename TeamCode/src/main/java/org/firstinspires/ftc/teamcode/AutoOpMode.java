package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Camera;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.ConceptScanServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.RobotConstants.GATE_UP_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.TEAM_MARKER_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.TEAM_MARKER_UP_POSITION;


public abstract class AutoOpMode extends LinearOpMode{

    //Declare all Motors, Servos, Sensors, etc
    // does it work????????????????????????????????????
    //Drive Train Motors

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor middleIntake;
    DcMotor outerIntake;
    DcMotor lift;
    DcMotor lift2;

    //Non-Drive Train Motors

    DcMotor intake;

    //Servos

    Servo TeamMarker;

    //Sensors

    BNO055IMU imu;

    //Other Variables

    ElapsedTime time; //returns time in s, ms, or ns since last reset

    double currentGyro;
    double previousGyro;
    int gyroMultiplier;
    double offset = -90;
    double reasonableRange = 0;
    double previousReasonableRange = 0;
    int numTurns = 0;

    ColorSensor CS;
    DistanceSensor DS;

     ModernRoboticsI2cRangeSensor rangeSensor;

    Servo basketServo;
    Servo gateServo;

    //ModernRoboticsI2cRangeSensor rangeSensor;


    //Constants

    double speedUp = 0;
    String[] vision = {"", "", "", "", "", "", "", "", "", "",};
    //Initializes Motors, Servos, Sensors, etc when the robot is hanging
    public void initialize() throws InterruptedException{
        time =  new ElapsedTime();
        //Drive Train Motors

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        //TeamMarker = hardwareMap.servo.get("TeamMarker");
        //Configures the encoders for the motors

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        middleIntake = hardwareMap.dcMotor.get("middleIntake");
        outerIntake = hardwareMap.dcMotor.get("outerIntake");
        lift = hardwareMap.dcMotor.get("lift");
        lift2 = hardwareMap.dcMotor.get("lift2");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        basketServo = hardwareMap.servo.get("basketServo");
        gateServo = hardwareMap.servo.get("gateServo");

        //Non-Drive Train Motors

        //Servos

        //Sensors

        initializeGyro();

        //Other Variables

        //CS = hardwareMap.colorSensor.get("goldDetector");
        //DS = hardwareMap.get(DistanceSensor.class, "goldDetector");

        rangeSensor = (ModernRoboticsI2cRangeSensor) hardwareMap.get(DistanceSensor.class, "sensor_range");

        //revRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        resetTimer();
        previousGyro = 0;
        currentGyro = 0;
        gyroMultiplier = 0;

        setStartServoPosition();

        telemetry.addData("Ready", "Freddy");
        telemetry.update();
    }

    public void setStartServoPosition() throws InterruptedException{
        basketServo.setPosition(TEAM_MARKER_DOWN_POSITION);
        gateServo.setPosition(GATE_UP_POSITION);
    }

    public void initializeDriveTrainOnly() throws InterruptedException{

        time =  new ElapsedTime();
        //Drive Train Motors

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        initializeGyro();

        resetTimer();
        previousGyro = 0;
        currentGyro = 0;
        gyroMultiplier = 0;
    }



    //Initializes Gyro, since in the actual game the gyro may need to be initialized after
    //everything else (once the robot lands on the ground)
    public void initializeGyro() throws InterruptedException{

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public double getGyroPitch() throws InterruptedException {
        Orientation angles = imu.getAngularOrientation();
        return (angles.secondAngle * -1);
    }

    public double getGyroRoll() throws InterruptedException {
        Orientation angles = imu.getAngularOrientation();
        return (angles.thirdAngle * -1);
    }

    public double getGyroYaw() throws InterruptedException {
        Orientation angles = imu.getAngularOrientation();
        return (angles.firstAngle * -1);
    }

    public int getAvgEncoder() throws InterruptedException {
        return ( ( Math.abs(BL.getCurrentPosition()) + Math.abs(BR.getCurrentPosition() + FR.getCurrentPosition() + FL.getCurrentPosition()) ) / 4);
    }

    public int getSingleEncoder() throws InterruptedException{
        return FR.getCurrentPosition();
    }

    public void PEncoderSetPowerForward(int encoderDist) throws InterruptedException{
        int startEnc = getSingleEncoder();
        double kP = 0.1 / 200;
        double kI = 0.07 / 1000;
        double base = 0.07;
        double startTime = System.currentTimeMillis();
        while (Math.abs(getSingleEncoder() - startEnc) < encoderDist && opModeIsActive()){
            int error = Math.abs(encoderDist - (getSingleEncoder() - startEnc));
            telemetry.addData("Error", error);
            double timeElapsedMS = System.currentTimeMillis() - startTime;
            double power = error * kP + base + kI * timeElapsedMS;
            //if (power > 0.4){
                power = 0.35;
            //}
            telemetry.addData("Power", power);
            setPower(power);
            telemetry.update();
        }
    }

    public void PEncoderSetPowerBackward(int encoderDist) throws InterruptedException{
        int startEnc = getSingleEncoder();
        double kP = 0.1 / 200;
        double kI = 0.07 / 1000;
        double base = 0.07;
        double startTime = System.currentTimeMillis();
        while (Math.abs(getSingleEncoder() - startEnc) < encoderDist && opModeIsActive()){
            int error = Math.abs(encoderDist - (getSingleEncoder() - startEnc));
            telemetry.addData("Error", error);
            double timeElapsedMS = System.currentTimeMillis() - startTime;
            double power = error * kP + base + kI * timeElapsedMS;
            if (power > 0.4){
                power = 0.4;
            }
            telemetry.addData("Power", -power);
            setPower(-power);
            telemetry.update();
        }
    }







    public void turn(double rotation, double angle) throws InterruptedException {
        double first = getFunctionalGyroYaw();
        while ((Math.abs(getFunctionalGyroYaw() - first) < angle) && (opModeIsActive())) {
            turn(rotation);
            idle();
        }
        setZero();
    }


    public void resetTimer() throws InterruptedException{
        time.reset();
    }

    public double getTime() throws InterruptedException{
        return time.milliseconds();
    }

    //Normally getGyroYaw() returns a value from between -180 to 180
    //This leads to issues when you are near -180 or 180
    //getFunctionalGyroYaw() effectively increases the range
    //to -infinity to infinity as long as it is constantly being
    //updated with gyro data
    public double getFunctionalGyroYaw() throws InterruptedException{
        previousGyro = currentGyro;
        offset = 0;
        currentGyro = getGyroYaw();
        if (previousGyro <= -160 && currentGyro > 160){
            gyroMultiplier--;
        }
        if (currentGyro <= -160 && previousGyro > 160){
            gyroMultiplier++;
        }
        return gyroMultiplier * 360 + getGyroYaw() + offset;
    }

    //TODO: Create basic methods to set all motors to a certain power + stop the motors

    public void setPower(double power) throws InterruptedException{
        FL.setPower(-power * .87);
        FR.setPower(power);
        BL.setPower(-power * .87);
        BR.setPower(power);
    }

    public void setPowerAndTurn(double power, double turn) throws InterruptedException{
        FL.setPower(-power + turn);
        FR.setPower(power + turn);
        BL.setPower(-power + turn);
        BR.setPower(power + turn);
    }

    public double findAnglularPositionError(double desiredAngle) throws InterruptedException{
        return getFunctionalGyroYaw() - desiredAngle;
    }

    public double simpleStraighten(double desiredAngle) throws InterruptedException{
        double error = findAnglularPositionError(desiredAngle);
        return error * 0.05;
    }

    public double simpleStraighten(double desiredAngle, double constant) throws InterruptedException{
        double error = findAnglularPositionError(desiredAngle);
        return error * constant;
    }

    public void moveForwardStraight(double power, double desiredAngle, int timeMS) throws InterruptedException{
        setPower(power);
        double startTime = time.milliseconds();
        while (time.milliseconds() - startTime < timeMS && opModeIsActive()){
            double correctionalTurn = simpleStraighten(desiredAngle);
            setPowerAndTurn(power, correctionalTurn);
            telemetry.addData("ANgle", getFunctionalGyroYaw());
            telemetry.update();
            idle();
        }
        setZero();
    }

    public void moveForwardStraightUntilObjectDetected(double power, double desiredAngle, int maxTimeMS) throws InterruptedException{
        setPower(power);
        double startTime = time.milliseconds();
        while (time.milliseconds() - startTime < maxTimeMS && !(DS.getDistance(DistanceUnit.CM) <= 12) && opModeIsActive()){
            double correctionalTurn = simpleStraighten(desiredAngle);
            setPowerAndTurn(power, correctionalTurn);
            telemetry.addData("Angle", getFunctionalGyroYaw());
            telemetry.update();
            idle();
        }

        setZero();
    }

    public void alignWithSample() throws InterruptedException{
        if (DS.getDistance(DistanceUnit.CM) >= 12 && opModeIsActive()){
            while (DS.getDistance(DistanceUnit.CM) >= 12 && opModeIsActive()){
                setPower(-0.12);
                idle();
            }
            setZero();
        }

    }


    public void setZero() throws InterruptedException{
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void turn(double power) throws InterruptedException{
        FL.setPower(power );
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
    }

    public void turnR(double power) throws InterruptedException{
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
    }

    //TODO: Create a basic time-based movement method

    public void moveTime(int time, double power) throws InterruptedException {

        setPower(power);
        sleep(time);
        setZero();
    }


    //TODO: Create a basic encoder-based movement method

    // Move forward based off encoders



    public void pMoveForward(int distance) throws InterruptedException {
        int startPos = getAvgEncoder();
        while ((Math.abs(getAvgEncoder() - startPos) < distance) && (opModeIsActive())) {
            int distanceAway = Math.abs(distance - Math.abs(getAvgEncoder() - startPos));
            setPower(distanceAway * .0005 + .24);
            telemetry.addData("distance", Math.abs(distanceAway - Math.abs(getAvgEncoder() - startPos)));
            telemetry.addData("Power", distanceAway * .0005 + .24);
            telemetry.update();
            idle();
        }
        setZero();
        if (Math.abs(getAvgEncoder() - startPos) > distance + 50) {
            telemetry.addData("overshoot", "fix");
            telemetry.update();
        }
    }

    public void pMoveBackward(int distance) throws InterruptedException {
        int startPos = getAvgEncoder();
        while ((Math.abs(getAvgEncoder() - startPos) < distance) && (opModeIsActive())) {
            int distanceAway = Math.abs(distance - Math.abs(getAvgEncoder() - startPos));
            setPower(-distanceAway * .0005 - .24);
            telemetry.addData("distance", Math.abs(distanceAway - Math.abs(getAvgEncoder() - startPos)));
            telemetry.addData("Power", distanceAway * .0005 - .24);
            telemetry.update();
            idle();
        }
        setZero();
        if (Math.abs(getAvgEncoder() - startPos) > distance + 50) {
            telemetry.addData("overshoot", "fix");
            telemetry.update();
        }
    }

    // Move forwards based off encoders
    public void moveForwardEncoder(double power, int distance) throws InterruptedException {
        int startPos = getAvgEncoder();
        while ((Math.abs(getAvgEncoder() - startPos) < distance) && (opModeIsActive())) {
            setPower(power);
            telemetry.addData("distance", getAvgEncoder() - startPos);
            telemetry.update();
            idle();
        }
        setZero();
        if (Math.abs(getAvgEncoder() - startPos) > distance + 50) {
            telemetry.addData("overshoot", "fix");
            telemetry.update();
        }
    }

    public void moveForwardEncoderP(int distance) throws InterruptedException {
        double startPos = getAvgEncoder();
        while ((Math.abs(getAvgEncoder() - startPos) < distance) && (opModeIsActive())) {
            double power = (0.15 / 500) * Math.abs(getAvgEncoder() - startPos) / distance + 0.14;
            if (power > 0.3){
                power = 0.3;
            }
            setPower(power);
            telemetry.addData("distance", getAvgEncoder() - startPos);
            telemetry.update();
            idle();
        }
        setZero();
        if (Math.abs(getAvgEncoder() - startPos) > distance + 50) {
            telemetry.addData("overshoot", "fix");
            telemetry.update();
        }
    }

    // Move backwards based off encoders
    public void moveBackwardEncoder(double power, int distance) throws InterruptedException {
        int startPos = getAvgEncoder();
        while ((Math.abs(getAvgEncoder() - startPos) < distance) && (opModeIsActive())) {
            setPower(-power);
            telemetry.addData("distance", getAvgEncoder() - startPos);
            telemetry.update();
            idle();
        }
        setZero();
        if (Math.abs(getAvgEncoder() - startPos) > distance + 50) {
            telemetry.addData("overshoot", "fix");
            telemetry.update();
        }
    }

    public void moveBackwardEncoderP(int distance) throws InterruptedException {
        double startPos = getAvgEncoder();
        while ((Math.abs(getAvgEncoder() - startPos) < distance) && (opModeIsActive())) {
            double power = -(0.15 / 350) * Math.abs(distance - (getAvgEncoder() - startPos)) - 0.08;
            telemetry.addData("Power", power);
            if (power < -0.4){
                power = -0.4;
            }
            setPower(power);
            telemetry.addData("distance", getAvgEncoder() - startPos);
            telemetry.update();
            idle();
        }
        setZero();
        if (Math.abs(getAvgEncoder() - startPos) > distance + 50) {
            telemetry.addData("overshoot", "fix");
            telemetry.update();
        }
    }

    //TODO: Create a Proportion-based encoder movement method

    //TODO: Create a PI or PID-based movement method

    //TODO: Create a basic time-based turning method

    //TODO: Create a Proportion-based turning method

    public void pRightTurn(double desired) throws InterruptedException {
        double start = getFunctionalGyroYaw();
        double proximity = Math.abs(desired);

        double speedUp = 0;
        double initialPower = 0.12;
        if (desired < 15){
            initialPower = 0.24;
        }
        double startTime = time.milliseconds();
        while (Math.abs(getFunctionalGyroYaw() - start) < desired - 1 && opModeIsActive()) {
            proximity = Math.abs((Math.abs(getFunctionalGyroYaw() - start) - desired));
            //double pTerm = proximity * 0.00355;
            double pTerm = proximity * 0.00455;
            if (pTerm > 0.45){
                pTerm = 0.45;
            }
            double power = -pTerm - initialPower - speedUp;

            telemetry.addData("Proximity Value: ", proximity);
            telemetry.addData("Yaw (Functional): ", getFunctionalGyroYaw());
            telemetry.addData("Time", time.milliseconds() - startTime);
            telemetry.update();
            //if (power > 0.4){
            //    power = 0.4;
            //}
            telemetry.addData("Turn Power: ", power);
            turn(power);
            speedUp = (time.milliseconds() - startTime) * 0.12 / 1000;
            if (time.milliseconds() - startTime > 3500){
                numTurns = 99;
                break;
            }
            //idle();
        }
        setZero();

        telemetry.update();
        //sleep(5000);
    }

    public void pLeftTurn(double desired) throws InterruptedException {
        double start = getFunctionalGyroYaw();
        double proximity = Math.abs(desired);

        double timeSinceSpeedIncrease = 0;
        speedUp = 0;
        double initialPower = 0.12;
        if (desired < 15){
            initialPower = 0.24;
        }
        double startTime = time.milliseconds();

        while (Math.abs(getFunctionalGyroYaw() - start) < desired - 1 && opModeIsActive()) {
            proximity = Math.abs((Math.abs(getFunctionalGyroYaw() - start) - desired));
            //double pTerm = proximity * 0.00355;
            double pTerm = proximity * 0.00455;
            if (pTerm > 0.45){
                pTerm = 0.45;
            }
            double power = pTerm + initialPower + speedUp;
            telemetry.addData("Proximity Value: ", proximity);

            telemetry.addData("Yaw Value:", getFunctionalGyroYaw());
            telemetry.addData("Speed Up", speedUp);
            telemetry.update();
            //if (power > 0.5){
            //    power = 0.5;
            //}
            telemetry.addData("Turn value: ", power);
            turn(power);
            //if (time.milliseconds() - startTime >= 1000) {
                speedUp = (time.milliseconds() - startTime) * 0.12 / 1000;
          //  }
            if (time.milliseconds() - startTime > 3500){
                numTurns = 99;
                break;
            }
            //idle();
        }
        setZero();
        telemetry.addData("Power", 0.2 + speedUp);
        telemetry.update();
        //sleep(5000);
    }


    public void pLeftTurnTest(double desired) throws InterruptedException {
        double start = getFunctionalGyroYaw();
        double proximity = Math.abs(desired);

        double timeSinceSpeedIncrease = 0;
        double startTime = time.milliseconds();
        speedUp = 0;
        double initialPower = 0.04;
        while (Math.abs(getFunctionalGyroYaw() - start) < desired - 1 && opModeIsActive()) {
            proximity = Math.abs((Math.abs(getFunctionalGyroYaw() - start) - desired));
            //double pTerm = proximity * 0.00355;
            double pTerm = proximity * 0.00455;
            if (pTerm > 0.4){
                pTerm = 0.4;
            }
            double power = pTerm + initialPower + speedUp;
            telemetry.addData("Proximity Value: ", proximity);

            telemetry.addData("Yaw Value:", getFunctionalGyroYaw());
            telemetry.addData("Speed Up", speedUp);
            telemetry.update();
            if (power > 0.4){
                power = 0.4;
            }
            telemetry.addData("Turn value: ", power);
            setPowerAndTurn(power, 0);
            //if (time.milliseconds() - startTime >= 1000) {
            speedUp = (time.milliseconds() - startTime) * 0.14 / 1000;
            //  }
            if (time.milliseconds() - startTime > 5000){
                break;
            }
            idle();
        }
        setZero();
        telemetry.addData("Power", 0.2 + speedUp);
        telemetry.update();
        //sleep(5000);
    }

    public void pRightTurnTest(double desired) throws InterruptedException {
        double start = getFunctionalGyroYaw();
        double proximity = Math.abs(desired);
        double startTime = time.milliseconds();
        double timeSinceSpeedIncrease = 0;
        speedUp = 0;
        double initialPower = 0.18;
        if (Math.abs(desired) > 30){
            initialPower = 0.13;
        }
        if (Math.abs(desired) > 60){
            initialPower = 0.08;
        }
        while (Math.abs(getFunctionalGyroYaw() - start) < desired - 1 && opModeIsActive()) {
            proximity = Math.abs((Math.abs(getFunctionalGyroYaw() - start) - desired));
            //double pTerm = proximity * 0.00355;
            double pTerm = proximity * 0.0043;
            if (pTerm > 0.3){
                pTerm = 0.3;
            }
            double power = pTerm + initialPower + speedUp;
            telemetry.addData("Proximity Value: ", proximity);

            telemetry.addData("Yaw Value:", getFunctionalGyroYaw());
            telemetry.addData("Speed Up", speedUp);
            telemetry.update();
            if (power > 0.4){
                power = 0.4;
            }
            telemetry.addData("Turn value: ", power);
            setPowerAndTurn(-0.04, -power);
            //if (time.milliseconds() - startTime >= 1000) {
            speedUp = (time.milliseconds() - startTime) * 0.14 / 1000;
            //  }
            if (time.milliseconds() - startTime > 5000){
                break;
            }
        }
        setZero();
        telemetry.addData("Power", 0.2 + speedUp);
        telemetry.update();
        //sleep(5000);
    }

    public void PILeftTurn(double desired) throws InterruptedException {
        double start = getFunctionalGyroYaw();
        double proximity = Math.abs(desired);
        double riemannSumError = 0;
        double kP = 0.0025;
        double kI = 0.0032 / 1000;
        double currentTime = System.currentTimeMillis();
        double pastTime;
        double time = 0;
        double initialPower = 0.0;
        if (desired > 90){
            kI = 0.0016 / 1000;
        }
        if (desired <= 90){
            initialPower = 0.15;
        }
        if (desired <= 30){
            initialPower = 0.25;
        }
        while (Math.abs(getFunctionalGyroYaw() - start) < desired && opModeIsActive()) {
            pastTime = currentTime;
            currentTime = System.currentTimeMillis();
            double deltaT = currentTime - pastTime;
            time += deltaT;
            //Convert time from ms to s
            telemetry.addData("Time", time / 1000.0);
            proximity = Math.abs((Math.abs(getFunctionalGyroYaw() - start) - desired));
            riemannSumError += deltaT * proximity;
            telemetry.addData("Proximity Value: ", proximity);
            telemetry.addData("Turn value: ", proximity * kP + kI * riemannSumError + 0.11);
            telemetry.addData("Yaw Value:", getFunctionalGyroYaw());
            telemetry.addData("Integral", kI * riemannSumError);
            telemetry.update();
            turn(proximity * kP + kI * riemannSumError + initialPower);
            idle();
        }
        setZero();

    }

    public void PIRightTurn(double desired) throws InterruptedException {
        double start = getFunctionalGyroYaw();
        double proximity = Math.abs(desired);
        double riemannSumError = 0;
        double kP = 0.0025;
        double kI = 0.0032 / 1000;
        double currentTime = System.currentTimeMillis();
        double pastTime;
        double time = 0;
        double initialPower = 0.0;
        if (desired > 90){
            kI = 0.0016 / 1000;
        }
        if (desired <= 90){
            initialPower = 0.15;
        }
        if (desired <= 30){
            initialPower = 0.25;
        }
        while (Math.abs(getFunctionalGyroYaw() - start) < desired && opModeIsActive()) {
            pastTime = currentTime;
            currentTime = System.currentTimeMillis();
            double deltaT = currentTime - pastTime;
            time += deltaT;
            //Convert time from ms to s
            telemetry.addData("Time", time / 1000.0);
            proximity = Math.abs((Math.abs(getFunctionalGyroYaw() - start) - desired));
            riemannSumError += deltaT * proximity;
            telemetry.addData("Proximity Value: ", proximity);
            telemetry.addData("Turn value: ", -proximity * kP - kI * riemannSumError - 0.11);
            telemetry.addData("Yaw Value:", getFunctionalGyroYaw());
            telemetry.addData("Integral", kI * riemannSumError);
            telemetry.update();
            turn(-proximity * kP - kI * riemannSumError - initialPower);
        }
        setZero();
    }

    public void turnToPosition(double desiredAngle) throws InterruptedException{
        if (getFunctionalGyroYaw() > desiredAngle){
            pLeftTurn(Math.abs(getFunctionalGyroYaw() - desiredAngle));
        }
        else if (getFunctionalGyroYaw() < desiredAngle){
            pRightTurn(Math.abs(getFunctionalGyroYaw() - desiredAngle));
        }
    }

    public void precisionTurnToPosition(double desiredAngle) throws InterruptedException{
        numTurns = 0;
        while (Math.abs(getFunctionalGyroYaw() - desiredAngle) > 3 && numTurns < 4 && opModeIsActive()){
            turnToPosition(desiredAngle);
            numTurns++;
        }
        numTurns = 0;
    }

    public void turnToPositionPI(double desiredAngle) throws InterruptedException{
        if (getFunctionalGyroYaw() > desiredAngle){
            PILeftTurn(Math.abs(getFunctionalGyroYaw() - desiredAngle));
        }
        else if (getFunctionalGyroYaw() < desiredAngle){
            PIRightTurn(Math.abs(getFunctionalGyroYaw() - desiredAngle));
        }
        turnToPosition(desiredAngle);
    }



    //TODO: Create a PI or PID-based turning method

    //TODO: Create basic methods to manipulate the addition servos and motors

    public void setTeamMarker() throws InterruptedException{
        TeamMarker.setPosition(TEAM_MARKER_UP_POSITION);
        telemetry.addData("TMarker Pos: ", TeamMarker.getPosition());
        telemetry.update();
    }

    public void dropTeamMarker() throws InterruptedException{
        gateServo.setPosition(0.5);
        basketServo.setPosition(0.7475);
        sleep(1000);
        //basketServo.setPosition(.4);
        //basketServo.setPosition(.9);
        //sleep(1000);
       // gateServo.setPosition(.7);
        //sleep(1000);
    }

    //TODO: Create an approach to detecting the gold. Some possibilities include Color Sensor, OpenCV, BitMaps

    //TODO: Create basic code for range sensors

    public double getRangeReading() throws InterruptedException{
        double reading = rangeSensor.getDistance(DistanceUnit.CM);
        while (reading > 400 || Double.isNaN(reading) && opModeIsActive()){
            reading = rangeSensor.getDistance(DistanceUnit.CM);
        }
        return reading;
    }

    public double[] getReasonableRangeTest() throws InterruptedException{
        double reading = rangeSensor.getDistance(DistanceUnit.MM);
        while (reading > 4000 || Double.isNaN(reading) && opModeIsActive()){
            reading = rangeSensor.getDistance(DistanceUnit.MM);
        }
        previousReasonableRange = reasonableRange;
        reasonableRange = reading;
        double[] arry = {reasonableRange, (reasonableRange - previousReasonableRange), previousReasonableRange};
        return arry;
    }

    //TODO: Create range sensor based movement code

    public void moveToRange(double rangeCM) throws InterruptedException {
        while (Math.abs(getRangeReading() - rangeCM) > 3 && opModeIsActive()){
            double error = getRangeReading() - rangeCM;
            if (error > 0){
                setPower(0.1 + Math.abs(error) * 0.17/60);
            }
            else if (error < 0){
                setPower(-0.1 - Math.abs(error) * 0.17/60);
            }
            telemetry.addData("Range", getRangeReading());
            telemetry.update();
            idle();
        }
        telemetry.addData("RangeMotion", "Complete");
        telemetry.update();
        setPower(0);
    }

    public void moveToRangeBasic(double rangeCM) throws InterruptedException {
        double kP = 0.23/63;
        double kI = 0.0000012;
        double currentTime = System.currentTimeMillis();
        double pastTime;
        double desired = getFunctionalGyroYaw();
        double time = 0;
        double numCalcs = 0;
        double riemannSumError = 0;
        double initialPower = 0.14;
        double error = 99;
        double prevError;
        while (Math.abs(getRangeReading() - rangeCM) > 3 && opModeIsActive()){
            prevError = error;
            error = getRangeReading() - rangeCM;
            double correctionalTurn = simpleStraighten(desired, 0.02);
            telemetry.addData("Error", error);
            telemetry.addData("Prev", prevError);
            if (Math.abs(error - prevError) < 0.05){
                telemetry.addData("Stagnation", "Stagnation");
            }
            pastTime = currentTime;
            currentTime = System.currentTimeMillis();
            double deltaT = currentTime - pastTime;
            time += deltaT;
            telemetry.addData("Time", time / 1000.0);
            numCalcs++;
            telemetry.addData("Count", numCalcs);
            riemannSumError += deltaT * (error);
            telemetry.addData("I Term", riemannSumError * kI);
            //setPower(error * 0.23/70 + riemannSumError * kI);
            double pTerm = kP * Math.abs(error);
            telemetry.addData("PTerm", pTerm);
            if (pTerm > 0.35){
                pTerm = 0.35;
            }
            double iTerm = time / 1000 * 0.05;

            if (iTerm > 0.15){
                iTerm = 0.15;
            }

            if (error > 0){
                //setPowerAndTurn(0 + pTerm + iTerm + 0.06, correctionalTurn);
            }
            else if (error < 0){
                //setPowerAndTurn(-0 - pTerm - iTerm - 0.06, correctionalTurn);
            }
            if (time > 3500){
                return;
            }
            telemetry.addData("Range", getRangeReading());
            telemetry.update();
            idle();
        }
        telemetry.addData("RangeMotion", "Complete");
        telemetry.update();
        setPower(0);
    }


    public void moveEncoderDistancePI(int distance) throws InterruptedException{
        int start = Math.abs(getAvgEncoder());
        double kp = .23/5000;
        double ki = .0000012;
        double realtime = 0;
        double change;
        double past = 0;
        double integral;
        double error = 0;
        while (Math.abs(getAvgEncoder()) - start < distance && opModeIsActive())
        {
            past = System.currentTimeMillis();
            change = (System.currentTimeMillis() - past);
            telemetry.addData("Change in time", change);
            realtime += change;
            error += (Math.abs(getAvgEncoder()) - start) * change;
            integral = error * ki;
            setPower((Math.abs(distance - getAvgEncoder()) * kp + .05) + integral );

        }
        setZero();

    }

    public void moveToRangeBasic(double rangeCM, double angle) throws InterruptedException {
        double kP = 0.23/63;
        double kI = 0.0000012;
        double currentTime = System.currentTimeMillis();
        double pastTime;
        double desired = angle;
        double time = 0;
        double numCalcs = 0;
        double riemannSumError = 0;
        double initialPower = 0.14;
        while (Math.abs(getRangeReading() - rangeCM) > 3 && opModeIsActive()){
            double error = getRangeReading() - rangeCM;
            double correctionalTurn = simpleStraighten(desired, 0.02);
            telemetry.addData("Error", error);
            pastTime = currentTime;
            currentTime = System.currentTimeMillis();
            double deltaT = currentTime - pastTime;
            time += deltaT;
            telemetry.addData("Time", time / 1000.0);
            numCalcs++;
            telemetry.addData("Count", numCalcs);
            riemannSumError += deltaT * (error);
            telemetry.addData("I Term", riemannSumError * kI);
            //setPower(error * 0.23/70 + riemannSumError * kI);
            double pTerm = kP * Math.abs(error);
            telemetry.addData("PTerm", pTerm);
            if (pTerm > 0.35){
                pTerm = 0.35;
            }
            double iTerm = time / 1000 * 0.05;

            if (iTerm > 0.15){
                iTerm = 0.15;
            }

            if (error > 0){
                setPowerAndTurn(0 + pTerm + iTerm + 0.06, correctionalTurn);
            }
            else if (error < 0){
                setPowerAndTurn(-0 - pTerm - iTerm - 0.06, correctionalTurn);
            }
            if (time > 3500){
                return;
            }
            telemetry.addData("Range", getRangeReading());
            telemetry.update();
            idle();
        }
        telemetry.addData("RangeMotion", "Complete");
        telemetry.update();
        setPower(0);
    }

    //Incorporates proportional and integral components to range sensor motion. Not as well tuned
    //as the version with just a P loop
    public void moveToRangePI(double rangeCM) throws InterruptedException {
        double kP = 0.23/70;
        double kI = 0.0000012;
        double currentTime = System.currentTimeMillis();
        double pastTime;
        double time = 0;
        double numCalcs = 0;
        double riemannSumError = 0;
        double initialPower = 0.05;
        if (Math.abs(getRangeReading() - rangeCM) < 10){
            initialPower = 0.09;
        }
        while (Math.abs(getRangeReading() - rangeCM) > 1.5 && opModeIsActive()){
            double error = getRangeReading() - rangeCM;
            double correctionalTurn = simpleStraighten(0, 0.02);
            telemetry.addData("Error", error);
            pastTime = currentTime;
            currentTime = System.currentTimeMillis();
            double deltaT = currentTime - pastTime;
            time += deltaT;
            telemetry.addData("Time", time / 1000.0);
            numCalcs++;
            telemetry.addData("Count", numCalcs);
            riemannSumError += deltaT * (error);
            telemetry.addData("I Term", riemannSumError * kI);
            //setPower(error * 0.23/70 + riemannSumError * kI);
            if (error > 0.5){
                setPower(initialPower + Math.abs(error) * 0.23/70 + riemannSumError * kI);
            }
            else if (error < -0.5){
                setPower(-initialPower - Math.abs(error) * 0.23/70 + riemannSumError * kI);
            }
            telemetry.addData("Range", getRangeReading());
            telemetry.update();
            idle();
        }
        setPower(0);
    }

    //Incorporates proportional and integral components to range sensor motion. Not as well tuned
    //as the version with just a P loop
    public void moveToRangePIStraighten(double rangeCM, double desiredAngle) throws InterruptedException {
        double kP = 0.18/50;
        double kI = 0.0010 / 1000;
        //double desiredAngle = getFunctionalGyroYaw();
        double currentTime = System.currentTimeMillis();
        double pastTime;
        double time = 0;
        double numCalcs = 0;
        double riemannSumError = 0;
        double initialPower = 0.04;
        double movementCase = 0;
        if (Math.abs(getRangeReading() - rangeCM) <= 50){
            initialPower = 0.1;
        }
        if (Math.abs(getRangeReading() - rangeCM) <= 15){
            initialPower = 0.15;
        }
        while (Math.abs(getRangeReading() - rangeCM) > 1.5 && opModeIsActive()){
            double error = getRangeReading() - rangeCM;
            double correctionalTurn = simpleStraighten(desiredAngle, 0.02);
            telemetry.addData("Error", error);
            pastTime = currentTime;
            currentTime = System.currentTimeMillis();
            double deltaT = currentTime - pastTime;
            time += deltaT;
            telemetry.addData("Time", time / 1000.0);
            numCalcs++;
            telemetry.addData("Count", numCalcs);
            riemannSumError += deltaT * (error);
            telemetry.addData("I Term", riemannSumError * kI);
            //setPower(error * 0.23/70 + riemannSumError * kI);
            if (error > 0.5){
                double power = initialPower + Math.abs(error) * kP + riemannSumError * kI;
                if (power > 0.25){
                    power = 0.25;
                }
                telemetry.addData("Power", power);
                setPowerAndTurn(power, correctionalTurn);
                if (movementCase == 0){
                    movementCase = 1;
                }
                if (movementCase == 2){
                    kI = 0;
                    initialPower = 0.11;
                    telemetry.addData("Reversing", "Direction");
                }
            }
            else if (error < -0.5){
                double power = -initialPower - Math.abs(error) * kP + riemannSumError * kI;
                if (power < -0.25){
                    power = -0.25;
                }
                telemetry.addData("Power", power);
                setPowerAndTurn(power, correctionalTurn);
                if (movementCase == 0){
                    movementCase = 2;
                }
                if (movementCase == 1){
                    kI = 0;
                    initialPower = 0.11;
                    telemetry.addData("Reversing", "Direction");
                }
            }
            if (time > 5000){
                break;
            }
            telemetry.addData("Range", getRangeReading());
            telemetry.update();
            idle();
        }
        setPower(0);
    }

    //Incorporates proportional and integral components to range sensor motion. Not as well tuned
    //as the version with just a P loop
    public void moveToRangePIStraightenToStartAngle(double rangeCM) throws InterruptedException {
        double kP = 0.18/35;
        double kI = 0.0010 / 1000;
        double desiredAngle = getFunctionalGyroYaw();
        double currentTime = System.currentTimeMillis();
        double pastTime;
        double time = 0;
        double numCalcs = 0;
        double riemannSumError = 0;
        double initialPower = 0.04;
        double movementCase = 0;
        if (Math.abs(getRangeReading() - rangeCM) <= 50){
            initialPower = 0.12;
        }
        if (Math.abs(getRangeReading() - rangeCM) <= 15){
            initialPower = 0.17;
        }
        while (Math.abs(getRangeReading() - rangeCM) > 1.5 && opModeIsActive()){
            double error = getRangeReading() - rangeCM;
            double correctionalTurn = simpleStraighten(desiredAngle, 0.03);
            telemetry.addData("Error", error);
            pastTime = currentTime;
            currentTime = System.currentTimeMillis();
            double deltaT = currentTime - pastTime;
            time += deltaT;
            telemetry.addData("Time", time / 1000.0);
            numCalcs++;
            telemetry.addData("Count", numCalcs);
            riemannSumError += deltaT * (error);
            telemetry.addData("I Term", riemannSumError * kI);
            //setPower(error * 0.23/70 + riemannSumError * kI);
            if (error > 0.5){
                double power = initialPower + Math.abs(error) * kP + riemannSumError * kI;
                if (power > 0.3){
                    power = 0.3;
                }
                telemetry.addData("Power", power);
                setPowerAndTurn(power, correctionalTurn);
                if (movementCase == 0){
                    movementCase = 1;
                }
                if (movementCase == 2){
                    kI = 0;
                    initialPower = 0.11;
                    telemetry.addData("Reversing", "Direction");
                }
            }
            else if (error < -0.5){
                double power = -initialPower - Math.abs(error) * kP + riemannSumError * kI;
                if (power < -0.3){
                    power = -0.3;
                }
                telemetry.addData("Power", power);
                setPowerAndTurn(power, correctionalTurn);
                if (movementCase == 0){
                    movementCase = 2;
                }
                if (movementCase == 1){
                    kI = 0;
                    initialPower = 0.11;
                    telemetry.addData("Reversing", "Direction");
                }
            }
            if (time > 5000){
                break;
            }
            telemetry.addData("Range", getRangeReading());
            telemetry.update();
            idle();
        }
        setPower(0);
    }

    //Move to a certain distance as perceived by the range sensor (can do both backwards and forwards)
    //while maintaining a certain angle. Generally this angle is the angle you start this motion as
    //since you want to maintain straightenness. However, it is possible to say set angle to 30 when
    //you're current angle is 0 if you wish to turn to that angle during the motion. However, this
    //may conflict with the range sensor's reading since the reading will change as you turn
    public void moveToRangeStraighten(double rangeCM, double angle) throws InterruptedException {
        while (Math.abs(getRangeReading() - rangeCM) > 2.5 && opModeIsActive()){
            double error = getRangeReading() - rangeCM;
            double angularCorrection = simpleStraighten(angle);
            if (error > 1){
                setPowerAndTurn(0.1 + Math.abs(error) * 0.23/50, angularCorrection);
            }
            else if (error < -1){
                setPowerAndTurn(-0.1 - Math.abs(error) * 0.23/50, angularCorrection);
            }
            telemetry.addData("Range", getRangeReading());
            telemetry.update();
            idle();
        }
        setPower(0);
    }


    //TODO: Create a potential gyro-based movement code to finish in the crater




    //Most likely not for drivetrain usage
    //Could be useful if we want to maintain a lift motor at a constant speed; however, this really
    //is experimental and not very accurate
    public void setConstantSpeed(DcMotor motor, double encoderPerSec) throws InterruptedException {
        double currentEnc = motor.getCurrentPosition();
        double pastEnc;
        double pastTime;
        double time = 0;
        double error;
        double count = 0;
        double totalSpeed = 0;
        double currentTime = System.currentTimeMillis();
        double power = -getCubicApproximationOfPower(encoderPerSec);
        telemetry.addData("Power", power);
        telemetry.update();
        sleep(3000);
        setPower(power);
        sleep(1000);
        while (opModeIsActive()){
            setPower(power);
            pastEnc = currentEnc;
            currentEnc = motor.getCurrentPosition();
            pastTime = currentTime;
            currentTime = System.currentTimeMillis();
            double deltaT = currentTime - pastTime;
            time += deltaT;
            double speed = (currentEnc - pastEnc) / (deltaT/1000);
            totalSpeed += speed;
            count++;
            telemetry.addData("Time", time);
            telemetry.addData("Speed", speed);
            telemetry.addData("Power", power);
            telemetry.addData("Average Speed", totalSpeed/count);
            telemetry.update();
            sleep(200);
            error = speed - encoderPerSec;
            power = power + 0.0000065 * error;
            idle();
        }
    }

    //Cubic Approximation of the relationship between encoder ticks per second and motor power
    //Generally with 10-20% of the actual value due to battery voltage variance and low sample size
    public double getCubicApproximationOfPower(double encoderpersec) throws InterruptedException{
        double x = encoderpersec;
        double approximatePower = 0.000000000116180 * Math.pow(x,3) - 0.000000372409239 * Math.pow(x,2) + 0.000447364393957 * x - 0.011674613712852;
        return approximatePower;
    }



    //Motion for moving into the crater. Allows robot to move essentially elliptically backwards by
    //altering its angle throughout the motion (if you need to visualize this, the square root function
    //is a pretty good comparison. This elliptical motion allows to avoid hitting the other alliance's
    //minerals and also ensures we don't approach the wall at a sharp angle.
    //A potential change to this could be experimenting with the function used to determined the
    //desired straightening angle
    public void glideAgainstWallMovingBack() throws InterruptedException{
        turnToPosition(55);
        setPower(-0.5);
        sleep(10000);
        /*
        double startTime = time.milliseconds();
        double initialAngle = getFunctionalGyroYaw();
        while (time.milliseconds() - startTime < 2500 && opModeIsActive()){
            double correctionalTurn = simpleStraighten(-(time.milliseconds() - startTime) * 20 / 2500 + initialAngle, 0.05);
            setPowerAndTurn(-0.29, correctionalTurn);
            telemetry.addData("Correctional Turn", correctionalTurn);
            telemetry.addData("Angle", getFunctionalGyroYaw());
            telemetry.update();
        }
        //It should reach the crater around here
        goGyroPark();
        sleep(5000);
        setZero();
        */
    }

    public void glideAgainstWallMovingBack(double angularChange, int timeMS) throws InterruptedException{
        double startTime = time.milliseconds();
        double initialAngle = getFunctionalGyroYaw();
        while (time.milliseconds() - startTime < timeMS && opModeIsActive()){
            double correctionalTurn = simpleStraighten(-(time.milliseconds() - startTime) / timeMS * angularChange + initialAngle, 0.05);
            setPowerAndTurn(-0.29, correctionalTurn);
            telemetry.addData("Correctional Turn", correctionalTurn);
            telemetry.addData("ANgle", getFunctionalGyroYaw());
            telemetry.update();
            idle();
        }
        //It should reach the crater around here
        setPower(-0.35);
        sleep(5000);
        setZero();
    }

    public void scoreMarker() throws InterruptedException{
        setTeamMarker();
        sleep(1000);
        dropTeamMarker();
        sleep(1000);
        setTeamMarker();
    }

    public void goGyroPark() throws InterruptedException {
        double power = 0;

        telemetry.addData("Gyro Pitch", getGyroPitch());
        telemetry.addData("power", power);
        telemetry.update();

        while(opModeIsActive()) {

            if (getGyroPitch() > 0 && getGyroPitch() < 1 ) {
                power = .46;
                setPower(power);
            }

            else if (getGyroPitch() < 5) {

                power =  (.225/(Math.abs(getGyroPitch())) + .1);
                setPower(power);
            }

            else {
                telemetry.addData("threshold", "motionFinished");
                telemetry.update();
                break;
            }
            idle();
        }
    }

    public void tallyVisionResults(String result) throws InterruptedException{
        if (result.equals("LEFT")){
            vision = shiftArrayDown(vision, "L");
        }
        if (result.equals("MIDDLE")){
            vision = shiftArrayDown(vision, "M");
        }
        if (result.equals("RIGHT")){
            vision = shiftArrayDown(vision, "R");
        }
        if (result.equals("UNKNOWN")){
            vision = shiftArrayDown(vision, "U");
        }
    }

    public String selectTarget() throws InterruptedException{
        int[] scores = {0, 0, 0};
        for (int i = 0; i < vision.length; i++){
            if (vision[i].equals("L")){
                scores[0] += (i+1);
            }
            if (vision[i].equals("M")){
                scores[1] += (i+1);
            }
            if (vision[i].equals("R")){
                scores[2] += (i+1);
            }
        }
        if (scores[0] > scores[1] && scores[0] > scores[2]){
            return "LEFT";
        }
        if (scores[2] > scores[1] && scores[2] > scores[0]){
            return "RIGHT";
        }
        return "MIDDLE";
    }

    public String[] shiftArrayDown(String[] array, String insertEnd) throws InterruptedException{
        for (int i = 0; i < array.length - 1; i++){
            array[i] = array[i + 1];
        }
        array[array.length - 1] = insertEnd;
        return array;
    }

    public String getTarget(String[] scores){
        int[] targetScores = {0, 0, 0};
        for (int i = 0; i < 10; i++){
            if (scores[i] != null) {
                if (scores[i].equals("L")) {
                    targetScores[0] += i;
                }
                if (scores[i].equals("M")) {
                    targetScores[1] += i;
                }
                if (scores[i].equals("R")) {
                    targetScores[2] += i;
                }
            }
        }
        if (targetScores[0] > targetScores[1] && targetScores[0] > targetScores[2]){
            return "Left";
        }
        else if (targetScores[1] > targetScores[2] && targetScores[1] > targetScores[0]){
            return "Middle";
        }
        return "Right";
    }

    public void rotateRobotWhileMovingForward(double power, int timeMS, double startAng, double finAng) throws InterruptedException{

        double coefficientM = Math.abs( (finAng - startAng) / timeMS);
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < timeMS && opModeIsActive()){
            setPowerAndTurn(power, startAng + coefficientM * (System.currentTimeMillis() - startTime));
        }
        setPower(0);
    }

    public void setPowerCorrectionalTurn(double power, double angle, int timeMS) throws InterruptedException{
        double startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < timeMS && opModeIsActive()){
            setPowerAndTurn(power, simpleStraighten(angle));
        }
        setPower(0);
    }

    public void raiseLift(double power) throws InterruptedException{
        lift.setPower(power);
        lift2.setPower(-power);
    }

    public int getLiftEncoder() throws InterruptedException{
        return lift.getCurrentPosition();
    }

    public void powerLiftUpTime(double power, double time) throws InterruptedException{
        double startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < time)){
            lift.setPower(power);
            lift2.setPower(-power);
        }
        lift.setPower(0);
        lift2.setPower(0);
    }
    public void powerLift(double power, int encDist) throws InterruptedException{
        int startEnc = getLiftEncoder();
        while (opModeIsActive() && (getLiftEncoder() - startEnc < encDist)){
            lift.setPower(power);
            lift2.setPower(-power);
        }
        lift.setPower(0);
        lift2.setPower(0);
    }

    public void powerLiftUpP(int encDist) throws InterruptedException{
        int startEnc = getLiftEncoder();
        while (opModeIsActive() && (getLiftEncoder() - startEnc < encDist)){
            int error = Math.abs(encDist - (getAvgEncoder() - startEnc));
            lift.setPower(0.1 + error * 0.15 / 200.0);
            lift2.setPower(-0.1 - error * 0.15 / 200.0);
            telemetry.addData("Power", 0.1 + error * 0.15 / 200.0);
            telemetry.update();
        }
        lift.setPower(0);
        lift2.setPower(0);
    }

    public void powerLiftDownP(int encDist) throws InterruptedException{
        int startEnc = getLiftEncoder();
        while (opModeIsActive() && (getLiftEncoder() - startEnc < encDist)){
            int error = Math.abs(encDist - (getAvgEncoder() - startEnc));
            lift.setPower(-0.1 - error * 0.15 / 200.0);
            lift2.setPower(0.1 + error * 0.15 / 200.0);
            telemetry.addData("Power", 0.1 + error * 0.15 / 200.0);
            telemetry.update();
        }
        lift.setPower(0);
        lift2.setPower(0);
    }

    public void delatch() throws InterruptedException{
        powerLiftUpP(400);
        powerLiftDownP(400);
    }






}
