package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


public class Drivetrain {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    public IMU imu;
    public GoBildaPinpointDriver odometry;

    public Servo lights;

    public void init(HardwareMap hardwareMap) {
        // drive
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        lights = hardwareMap.get(Servo.class, "LEDs");

        // Directions & modes
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

        double ODOMETRY_X_OFFSET = -102;
        double ODOMETRY_Y_OFFSET = -122;

        odometry.setOffsets(ODOMETRY_X_OFFSET, ODOMETRY_Y_OFFSET, DistanceUnit.MM);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.resetPosAndIMU();

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    public void drive(double y, double x, double rx, double accelerator) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
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

    public double getPosX() {
        return odometry.getPosX(DistanceUnit.INCH);
    }

    public double getPosY() {
        return odometry.getPosY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return odometry.getHeading(AngleUnit.DEGREES);
    }

    public void setHeading(double heading) { 
        odometry.setHeading(heading, AngleUnit.DEGREES);
    }

    public void setOdometryXY(double x, double y){
        odometry.setPosX(x, DistanceUnit.INCH);
        odometry.setPosY(y, DistanceUnit.INCH);
    }
    public void odometryUpdate() {
        odometry.update();
    }

    public void setLightPWM(double pwm){
        //0.0 off
        //0.277 red
        //0.333 orange
        //0.388 yellow
        //0.5 green
        //0.611 blue
        //0.722 violet
        //1.0 white
        lights.setPosition(pwm);
    }
    public void autoLight(boolean full){
        if (full){setLightPWM(0.5);}
        else{setLightPWM(0.277);}
    }
}

