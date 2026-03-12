package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.DcMotorMax;

public class Drivetrain {
    private final DcMotorMax frontLeft;
    private final DcMotorMax backLeft;
    private final DcMotorMax frontRight;
    private final DcMotorMax backRight;
    public IMU imu;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorMax.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorMax.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorMax.class, "frontRight");
        backRight = hardwareMap.get(DcMotorMax.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setCachingThreshold(0.01);
        backLeft.setCachingThreshold(0.01);
        frontRight.setCachingThreshold(0.01);
        backRight.setCachingThreshold(0.01);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    public void drive(double y, double x, double rx, double accelerator) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Deceleration
        double accelMultiplier = Math.min(1.0 - 0.7 * accelerator, 1.0);
        x *= 1.1;

        double flPower = accelMultiplier * ((y + x + rx) / denominator);
        double blPower = accelMultiplier * ((y - x + rx) / denominator);
        double frPower = accelMultiplier * ((y - x - rx) / denominator);
        double brPower = accelMultiplier * ((y + x - rx) / denominator);

        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);
    }

    public void resetFieldCentric() {
        imu.resetYaw();
    }

    public void driveFieldCentric(double y, double x, double rx, double accelerator) {
        double imuBotHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(imuBotHeading) - y * Math.sin(imuBotHeading);
        double rotY = x * Math.sin(imuBotHeading) + y * Math.cos(imuBotHeading);
        drive(rotY, rotX, rx, accelerator);
    }
}
