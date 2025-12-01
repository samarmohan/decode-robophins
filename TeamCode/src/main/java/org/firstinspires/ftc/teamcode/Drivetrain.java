package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class Drivetrain {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private GoBildaPinpointDriver odometry;

    private Limelight3A limelight;

    private double headingOffset;

    private double llx;

    private double lly;

    private double llh;

    private final double METER_TO_INCH = 39.37;

    public void init(HardwareMap hardwareMap) {
        // drive
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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

        double ODOMETRY_X_OFFSET = 17.6333602247;
        double ODOMETRY_Y_OFFSET = -150.236697005;

        odometry.setOffsets(ODOMETRY_X_OFFSET, ODOMETRY_Y_OFFSET, DistanceUnit.MM);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.resetPosAndIMU();

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        headingOffset = 0;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
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
        headingOffset = -odometry.getHeading(AngleUnit.RADIANS);
    }

    public void driveFieldCentric(double y, double x, double rx, double accelerator) {
        double botHeading = -odometry.getHeading(AngleUnit.RADIANS) - headingOffset;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
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

    public void setHeading(double heading){ odometry.setHeading(heading, AngleUnit.DEGREES);}

    public void odometryUpdate() {
        odometry.update();
    }

    public void limelightUpdate(double heading, boolean isLimelightOn){
        limelight.updateRobotOrientation(heading);
        LLResult result = limelight.getLatestResult();

        if (result != null){
            if (result.isValid()){
                Pose3D botpose = result.getBotpose();
                 llx = METER_TO_INCH * botpose.getPosition().x;
                 lly = METER_TO_INCH * botpose.getPosition().y;
                 llh = botpose.getOrientation().getYaw();


                if (isLimelightOn){
                        Pose3D pose_mt2 = result.getBotpose_MT2();
                        double mt2x = METER_TO_INCH * pose_mt2.getPosition().x;
                        double mt2y = METER_TO_INCH * pose_mt2.getPosition().y;
                        odometry.setPosX(mt2y + 72, DistanceUnit.INCH);
                        odometry.setPosY((-mt2x) + 72, DistanceUnit.INCH);
                }
            }
        }
    }

    public void limelightStart(){
        limelight.start();
    }
    public double getLlx(){
        return llx;
    }
    public double getLly(){
        return lly;
    }
    public double getLlh(){
        return llh;
    }
}

