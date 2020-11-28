package org.firstinspires.ftc.teamcode.lmsbots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc16072.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import javafx.geometry.Pos;

@Autonomous(name = "Mechanum Autonomous Sandbox", group = "lmsbots")
public class MechanumAutonomousSandbox extends LinearOpMode {
    private DcMotor driveFL, driveFR, driveBL,driveBR;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle=0, localAngle=0, power = .50, correction;

    // global class variables for where the robot is located on x,y axis
    private int xPos = 0;
    private int yPos = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // function to do init routine
        initialize();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

//        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // maps imu variables to hardware configuration name and initializes imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        if (opModeIsActive())
        {
            // autonomous code goes here
            driveTo(-48,24);

            /*driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            driveFL.setTargetPosition(4000);
            driveFR.setTargetPosition(4000);
            driveBL.setTargetPosition(4000);
            driveBR.setTargetPosition(4000);

            driveFL.setPower(1);
            driveFR.setPower(1);
            driveBL.setPower(1);
            driveBR.setPower(1);

           while (opModeIsActive() && (driveFL.isBusy() || driveFR.isBusy()))   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            {
                telemetry.addData("encoder-fwd-left", driveFL.getCurrentPosition() + "  busy=" + driveFL.isBusy());
                telemetry.addData("encoder-back-left", driveBL.getCurrentPosition() + "  busy=" + driveFL.isBusy());
                telemetry.addData("encoder-fwd-right", driveFR.getCurrentPosition() + "  busy=" + driveFR.isBusy());
                telemetry.addData("encoder-back-right", driveBR.getCurrentPosition() + "  busy=" + driveFR.isBusy());
                telemetry.update();
                idle();
            }

            driveFL.setPower(0);
            driveFR.setPower(0);
            driveBL.setPower(0);
            driveBR.setPower(0);

            resetStartTime();*/

        }
    }

    // initialization class
    private void initialize() {

        // maps drive motor variables to hardware configuration names
        driveFL = hardwareMap.get(DcMotor.class, "front_left_motor");
        driveFR = hardwareMap.get(DcMotor.class, "front_right_motor");
        driveBL = hardwareMap.get(DcMotor.class, "back_left_motor");
        driveBR = hardwareMap.get(DcMotor.class, "back_right_motor");

        // sets right motors to reverse direction so they're going the right way
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);

        // sets drive motor zero power behavior to brake
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    public void driveTo(int xTarget, int yTarget){
        int wheelDiameter = 4;
        double wheelCircumference = wheelDiameter * Math.PI;
        int encoderTicksPerRotation = 1125;
        double encoderTicksPerInch = encoderTicksPerRotation / wheelCircumference;
        int xDiff = xTarget - xPos;
        int yDiff = yTarget - yPos;
        double heading;

        if (xDiff >=0){
            heading = Math.toDegrees(Math.atan2(xDiff,yDiff));
        }
        else if (yDiff < 0){
            heading = 180 + Math.toDegrees(Math.atan2(Math.abs(xDiff), Math.abs(yDiff)));
        }
        else {
            heading = 270 + Math.toDegrees(Math.atan2(Math.abs(yDiff), Math.abs(xDiff)));
        }

        // determine distance needed to travel using pythagorean theorem
        double distance = Math.sqrt((xDiff*xDiff)+(yDiff*yDiff));
        double encoderTicks = encoderTicksPerInch * (Math.abs(xDiff) + Math.abs(yDiff));
        telemetry.addData("distance",(int)distance);
        telemetry.addData("heading",heading);
        telemetry.update();
        sleep(3000);

        encoderDrive((int)encoderTicks,1,(int)heading);
        xPos = xPos + xDiff;
        yPos = yPos + yDiff;
    }

    // method for driving by encoder
    private void encoderDrive(int encoderTicks, double power, int heading) {

        double powerFLBR = (power*FLBRpowerRatio(heading));
        double powerFRBL = (power*FRBLpowerRatio(heading));

        // encoder position has to be an integer
        int encoderFLBR = (int)(encoderTicks*powerFLBR);
        int encoderFRBL = (int)(encoderTicks*powerFRBL);

        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveFL.setTargetPosition(encoderFLBR);
        driveFR.setTargetPosition(encoderFRBL);
        driveBL.setTargetPosition(encoderFRBL);
        driveBR.setTargetPosition(encoderFLBR);

        driveFL.setPower(powerFLBR);
        driveFR.setPower(powerFRBL);
        driveBL.setPower(powerFRBL);
        driveBR.setPower(powerFLBR);

        while (opModeIsActive() && (driveFL.isBusy() || driveFR.isBusy()))   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left", driveFL.getCurrentPosition() + "  busy=" + driveFL.isBusy());
            telemetry.addData("encoder-back-left", driveBL.getCurrentPosition() + "  busy=" + driveFL.isBusy());
            telemetry.addData("encoder-fwd-right", driveFR.getCurrentPosition() + "  busy=" + driveFR.isBusy());
            telemetry.addData("encoder-back-right", driveBR.getCurrentPosition() + "  busy=" + driveFR.isBusy());
            telemetry.addData("xPos",xPos);
            telemetry.addData("yPos",yPos);
            telemetry.update();
            idle();
        }

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

        resetStartTime();
    }

    double FLBRpowerRatio(double heading){
        double powerRatio=0;

        while(heading>=360){
            heading = heading - 360;
        }

        if (heading>=0 && heading <= 90){
            powerRatio = 1;
        }
        else if (heading >= 180 && heading <=270){
            powerRatio = -1;
        }
        else if (heading == 135 || heading == 315){
            powerRatio = 0;
        }
        else if ((heading > 90 && heading < 135)){
            heading = heading - 90;
            powerRatio = 1-(heading/45);
        }
        else if ((heading > 315 && heading < 360)){
            heading = heading - 315;
            powerRatio = heading/45;
        }
        else if ((heading > 135 && heading < 180)){
            heading = heading - 135;
            powerRatio = -1 * (heading/45);
        }
        else if ((heading > 270 && heading < 315)){
            heading = heading - 270;
            powerRatio = (-1 * (1-(heading/45)));
        }

        return powerRatio;
    }

    double FRBLpowerRatio(double heading){
        double powerRatio=0;

        while(heading>=360){
            heading = heading - 360;
        }

        if (heading==0 || (heading >= 270 && heading < 360)){
            powerRatio=1;
        }
        else if (heading >= 90 && heading <=180){
            powerRatio = -1;
        }
        else if (heading == 45 || heading == 225) {
            powerRatio = 0;
        }
        else if ((heading > 0 && heading < 45)){
            powerRatio = 1-(heading/45);
        }
         else if ((heading > 225 && heading < 270)){
            heading = heading - 225;
            powerRatio = heading/45;
        }
        else if ((heading > 45 && heading < 90)){
            heading = heading - 45;
            powerRatio = -1 * (heading/45);
        }
        else if ((heading > 180 && heading < 225)){
            heading = heading - 180;
            powerRatio = (-1 * (1-(heading/45)));
        }

        return powerRatio;
    }

    // method for timed autonomous driving, accepts direction, motor power and time
    private void timedDrive(String direction, double power, int time) {
        if (direction.equals("forward")){
            driveFL.setPower(power);
            driveFR.setPower(power);
            driveBL.setPower(power);
            driveBR.setPower(power);

            sleep(time);

            driveFL.setPower(0);
            driveFR.setPower(0);
            driveBL.setPower(0);
            driveBR.setPower(0);
        }
        else if (direction.equals("backward")){
            driveFL.setPower(-power);
            driveFR.setPower(-power);
            driveBL.setPower(-power);
            driveBR.setPower(-power);

            sleep(time);

            driveFL.setPower(0);
            driveFR.setPower(0);
            driveBL.setPower(0);
            driveBR.setPower(0);
        }
        else if (direction.equals("strafeleft")){
            driveFL.setPower(-power);
            driveFR.setPower(power);
            driveBL.setPower(power);
            driveBR.setPower(-power);

            sleep(time);

            driveFL.setPower(0);
            driveFR.setPower(0);
            driveBL.setPower(0);
            driveBR.setPower(0);
        }
        else if (direction.equals("straferight")){
            driveFL.setPower(power);
            driveFR.setPower(-power);
            driveBL.setPower(-power);
            driveBR.setPower(power);

            sleep(time);

            driveFL.setPower(0);
            driveFR.setPower(0);
            driveBL.setPower(0);
            driveBR.setPower(0);
        }
    }

    // class to add and update telemetry
    private void composeTelemetry() {
        telemetry.addData("Front Left Drive Motor Power", driveFL.getPower());
        telemetry.addData("Front Right Drive Motor Power", driveFR.getPower());
        telemetry.addData("Back Left Drive Motor Power", driveBL.getPower());
        telemetry.addData("Back Right Drive Motor Power", driveBR.getPower());
        telemetry.update();
    }

    private void rotate(double degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        driveBL.setPower(leftPower);
        driveFL.setPower(leftPower);
        driveFR.setPower(rightPower);
        driveBR.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // for right turn we have to get off zero first
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn
            while (opModeIsActive() && getAngle() < degrees) {}


        // turn the motors off.
        driveFL.setPower(0);
        driveBL.setPower(0);
        driveFR.setPower(0);
        driveBR.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
//        resetAngle();
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
}



 /*   private Robot robot = new Robot();
    int state = 0;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
    }


    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override



    public void loop() {
        telemetry.addData("X", robot.nav.getEstimatedPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Y", robot.nav.getEstimatedPosition().getY(DistanceUnit.INCH));
        telemetry.addData("state", state);
        switch (state) {
            case 0:
                if (robot.nav.driveTo(24, 0, DistanceUnit.INCH)) {
                    state = 1;
                }
                break;
            case 1:
                if (robot.nav.rotateTo(90, AngleUnit.DEGREES)) {
                    state = 2;
                }
                break;
            case 2:
                if (robot.nav.driveTo(0, 0, DistanceUnit.INCH)) {
                    state = 3;
                }
                break;
            case 3:

                if (robot.nav.driveTo(-24, -24, DistanceUnit.INCH)) {
                    state = 4;
                }
                break;

        }
    }
}
*/