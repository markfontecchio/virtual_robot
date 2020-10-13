package org.firstinspires.ftc.teamcode.lmsbots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.ColumnMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;

public class MechanumDrive {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftBack;

    public static double GEAR_RATIO = 1.0; // for simulator - ours should be 0.5f;
    public static double WHEEL_RADIUS = 5.0;  // 5 cm
    public static double TICKS_PER_ROTATION = 1120.0;  // From NeveRest (for simulator)  GoBilda should be 383.6f

    public static double CM_PER_TICK = (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS) / TICKS_PER_ROTATION;

    private double maxSpeed = 1.0;

    private MatrixF conversion;
    private GeneralMatrixF encoderMatrix = new GeneralMatrixF(3, 1);

    private int frontLeftOffset;
    private int frontRightOffset;
    private int backRightOffset;
    private int backLeftOffset;


    MechanumDrive() {
        float[] data = {1.0f, 1.0f, 1.0f,
                1.0f, -1.0f, -1.0f,
                1.0f, -1.0f, 1.0f};
        conversion = new GeneralMatrixF(3, 3, data);
        conversion = conversion.inverted();
    }

    void init(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotor.class, "front_left_motor");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront = hwMap.get(DcMotor.class, "front_right_motor");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack = hwMap.get(DcMotor.class, "back_left_motor");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack = hwMap.get(DcMotor.class, "back_right_motor");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
    }

    private void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        double largest = maxSpeed;
        largest = Math.max(largest, Math.abs(flSpeed));
        largest = Math.max(largest, Math.abs(frSpeed));
        largest = Math.max(largest, Math.abs(blSpeed));
        largest = Math.max(largest, Math.abs(brSpeed));

        leftFront.setPower(flSpeed / largest);
        rightFront.setPower(frSpeed / largest);
        leftBack.setPower(blSpeed / largest);
        rightBack.setPower(brSpeed / largest);
    }

    void driveMechanum(double forward, double strafe, double rotate) {
        leftFront.setPower(forward + strafe + rotate);
        rightFront.setPower(forward - strafe - rotate);
        leftBack.setPower(forward - strafe + rotate);
        rightBack.setPower(forward + strafe - rotate);
//        double frontLeftSpeed = forward + strafe + rotate;
//        double frontRightSpeed = forward - strafe - rotate;
//        double backLeftSpeed = forward - strafe + rotate;
//        double backRightSpeed = forward + strafe - rotate;

//        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    double[] getDistanceCm() {
        double[] distances = {0.0, 0.0};

        encoderMatrix.put(0, 0, (float) ((leftFront.getCurrentPosition() - frontLeftOffset) * CM_PER_TICK));
        encoderMatrix.put(1, 0, (float) ((rightFront.getCurrentPosition() - frontRightOffset) * CM_PER_TICK));
        encoderMatrix.put(2, 0, (float) ((leftBack.getCurrentPosition() - backLeftOffset) * CM_PER_TICK));

        MatrixF distanceMatrix = conversion.multiplied(encoderMatrix);
        distances[0] = distanceMatrix.get(0, 0);
        distances[1] = distanceMatrix.get(1, 0);

        return distances;
    }

    void setMaxSpeed(double speed) {
        maxSpeed = Math.min(speed, 1.0);
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setEncoderOffsets() {
        frontRightOffset = rightFront.getCurrentPosition();
        frontLeftOffset = leftFront.getCurrentPosition();
        backLeftOffset = leftBack.getCurrentPosition();
        backRightOffset = rightBack.getCurrentPosition();
    }
}