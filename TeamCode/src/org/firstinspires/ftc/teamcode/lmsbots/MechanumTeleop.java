package org.firstinspires.ftc.teamcode.lmsbots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lmsbots.MechanumDrive;

import sun.security.mscapi.CPublicKey;


//@TeleOp(name = "Mechanum Teleop", group = "lmsbots")
public class MechanumTeleop extends LinearOpMode {

    // sets variables for drive motors
    private DcMotor driveFL, driveFR, driveBL, driveBR;

    // creates variables for drive inputs from controllers
    private double forwardBackward, leftRight, rotate;

    @Override
    public void runOpMode() {

        // calls initialize class to do init routine
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // sets values of variables for gamepad1 (start+a) inputs
            forwardBackward = -gamepad1.left_stick_y;  // uses left stick to move forward and backward
            leftRight = gamepad1.left_stick_x;  // uses left stick to strafe left and right
            rotate = gamepad1.right_stick_x;  // uses right stick to rotate

            driveFL.setPower(forwardBackward + leftRight + rotate);
            driveFR.setPower(forwardBackward - leftRight - rotate);
            driveBL.setPower(forwardBackward - leftRight + rotate);
            driveBR.setPower(forwardBackward + leftRight - rotate);

        }

    /*private MechanumDrive mechanumDrive = new MechanumDrive();
    private double[] distances;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        mechanumDrive.init(hardwareMap);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        mechanumDrive.driveMechanum(forward, strafe, rotate);
        distances = mechanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);
*/
    }

    // initialization class
    private void initialize() {

        // maps drive motor variables to hardware configuration names
        driveFL = hardwareMap.get(DcMotor.class, "front_left_motor");
        driveFR = hardwareMap.get(DcMotor.class, "front_right_motor");
        driveBL = hardwareMap.get(DcMotor.class, "back_left_motor");
        driveBR = hardwareMap.get(DcMotor.class, "back_right_motor");

        // sets left motors to reverse direction so they're going the correct way
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
}
