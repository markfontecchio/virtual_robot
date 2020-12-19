package org.firstinspires.ftc.teamcode.lmsbots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Autonomous Maze", group = "lmsbots")
public class AutonomousMaze extends LinearOpMode {
    private DcMotor driveFL, driveFR, driveBL,driveBR;
    BNO055IMU imu;
    double drivePower = 0.8;
    double turnPower = 0.6;
    int wheelDiameter = 4;
    int encoderTicksPerRotation = 1125;
    double wheelCircumference = wheelDiameter * Math.PI;
    double encoderTicksPerInch = encoderTicksPerRotation / wheelCircumference;
    double robotHeading = 0;
    int xPos = 0, yPos = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // method to initialize robot
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (when driver presses STOP)

        if (opModeIsActive())
        {
            driveTo(36, 48);
            //driveTo(-24,-24);

            // drive forward 48"
            //driveForward(48);

            // strafe right 36"
            //strafeRight(36);

        }
    }

    private void driveTo(int xTarget, int yTarget) {

        int xDiff = xTarget - xPos;
        int yDiff = yTarget - yPos;

        telemetry.addData("xPos", xPos);
        telemetry.addData("yPos", yPos);
        telemetry.update();
        sleep(4000);

        // move the robot
        driveForward(yTarget);
        strafeRight(xTarget);

        // update the current position of the robot
        // put some code here to update position of the robot

        telemetry.addData("xPos", xPos);
        telemetry.addData("yPos", yPos);
        telemetry.update();
        sleep(4000);

    }

    private void strafeRight(int inches) {
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveFL.setTargetPosition((int)(inches * encoderTicksPerInch));
        driveFR.setTargetPosition(-(int)(inches * encoderTicksPerInch));
        driveBL.setTargetPosition(-(int)(inches * encoderTicksPerInch));
        driveBR.setTargetPosition((int)(inches * encoderTicksPerInch));

        driveFL.setPower(drivePower);
        driveFR.setPower(-drivePower);
        driveBL.setPower(-drivePower);
        driveBR.setPower(drivePower);

        while (driveFL.isBusy()) {
            idle();
        }

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void strafeLeft(int inches) {
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveFL.setTargetPosition(-(int)(inches * encoderTicksPerInch));
        driveFR.setTargetPosition((int)(inches * encoderTicksPerInch));
        driveBL.setTargetPosition((int)(inches * encoderTicksPerInch));
        driveBR.setTargetPosition(-(int)(inches * encoderTicksPerInch));

        driveFL.setPower(-drivePower);
        driveFR.setPower(drivePower);
        driveBL.setPower(drivePower);
        driveBR.setPower(-drivePower);

        while (driveFL.isBusy()) {
            idle();
        }

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void driveBackward(int inches) {
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveFL.setTargetPosition(-(int)(inches * encoderTicksPerInch));
        driveFR.setTargetPosition(-(int)(inches * encoderTicksPerInch));
        driveBL.setTargetPosition(-(int)(inches * encoderTicksPerInch));
        driveBR.setTargetPosition(-(int)(inches * encoderTicksPerInch));

        driveFL.setPower(-drivePower);
        driveFR.setPower(-drivePower);
        driveBL.setPower(-drivePower);
        driveBR.setPower(-drivePower);

        while (driveFL.isBusy()) {
            idle();
        }

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private void driveForward(int inches) {

        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveFL.setTargetPosition((int)(inches * encoderTicksPerInch));
        driveFR.setTargetPosition((int)(inches * encoderTicksPerInch));
        driveBL.setTargetPosition((int)(inches * encoderTicksPerInch));
        driveBR.setTargetPosition((int)(inches * encoderTicksPerInch));

        driveFL.setPower(drivePower);
        driveFR.setPower(drivePower);
        driveBL.setPower(drivePower);
        driveBR.setPower(drivePower);

        while (driveFL.isBusy()) {
            idle();
        }

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private void turnRight(int degrees) {
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (robotHeading - degrees > -180) {
            while (opModeIsActive() && Angle() > (robotHeading - degrees)){
                driveFL.setPower(turnPower);
                driveFR.setPower(-turnPower);
                driveBL.setPower(turnPower);
                driveBR.setPower(-turnPower);
            }
        }
        else {
            while (opModeIsActive() && Angle() < (360 + (robotHeading - degrees))){
                driveFL.setPower(turnPower);
                driveFR.setPower(-turnPower);
                driveBL.setPower(turnPower);
                driveBR.setPower(-turnPower);
            }
            while (opModeIsActive() && Angle() > (360 + (robotHeading - degrees))){
                driveFL.setPower(turnPower);
                driveFR.setPower(-turnPower);
                driveBL.setPower(turnPower);
                driveBR.setPower(-turnPower);
            }
        }

        robotHeading = Angle();

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

    }

    private void turnLeft(int degrees) {
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (robotHeading + degrees > 180) {
            while (opModeIsActive() && Angle() > -(360 - (robotHeading + degrees))){
                driveFL.setPower(-turnPower);
                driveFR.setPower(turnPower);
                driveBL.setPower(-turnPower);
                driveBR.setPower(turnPower);
            }
            while (opModeIsActive() && Angle() < -(360 - (robotHeading + degrees))){
                driveFL.setPower(-turnPower);
                driveFR.setPower(turnPower);
                driveBL.setPower(-turnPower);
                driveBR.setPower(turnPower);
            }
        }
        else {
            while (opModeIsActive() && Angle() < (robotHeading + degrees)){
                driveFL.setPower(-turnPower);
                driveFR.setPower(turnPower);
                driveBL.setPower(-turnPower);
                driveBR.setPower(turnPower);
            }
        }

        robotHeading = Angle();

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);
    }

    private double Angle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // robot initialization method
    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // map our variables to robot components
        driveFL = hardwareMap.get(DcMotor.class, "front_left_motor");
        driveFR = hardwareMap.get(DcMotor.class, "front_right_motor");
        driveBL = hardwareMap.get(DcMotor.class, "back_left_motor");
        driveBR = hardwareMap.get(DcMotor.class, "back_right_motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);

        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

//        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

}