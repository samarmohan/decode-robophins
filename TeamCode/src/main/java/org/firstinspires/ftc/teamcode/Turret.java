package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    public DcMotorEx flywheel;
    public DcMotorEx flywheel2;
    public DcMotor frontIntake;
    public DcMotor backIntake;
    public Servo rightR;
    public Servo leftR;
    public Servo pitch;

    private static final double ENCODER_TICKS_PER_REV = 28.0;
    private static final double SECONDS_PER_MINUTE = 60.0;

    public static final double FLYWHEEL_RPM_FAST = 3400.0;
    public static final double FLYWHEEL_RPM_SLOW = 2800.0;

    enum RobotState {
        HOLD,
        OUTPUT,
        SHOOT
    }

    public void init(HardwareMap hardwareMap) {
        //rotation = hardwareMap.get(DcMotorEx.class, "rotation");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        flywheel.setVelocityPIDFCoefficients(300, 10, 1, 1.0);
        flywheel2.setVelocityPIDFCoefficients(300, 10, 1, 1.0);

        //rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontIntake.setDirection(DcMotor.Direction.REVERSE);

        pitch = hardwareMap.get(Servo.class, "pitch");
        rightR = hardwareMap.get(Servo.class, "rightR");
        leftR  = hardwareMap.get(Servo.class, "leftR");
        pitch.setDirection(Servo.Direction.REVERSE);
        rightR.setDirection(Servo.Direction.FORWARD);
        leftR.setDirection(Servo.Direction.FORWARD);
    }
    

    public void setFlywheelRPM(double rpm) {
        flywheel.setVelocity(rpm * (ENCODER_TICKS_PER_REV / SECONDS_PER_MINUTE));
        flywheel2.setVelocity(rpm * (ENCODER_TICKS_PER_REV / SECONDS_PER_MINUTE));
    }

    public double getFlywheelVelocity() {
        return SECONDS_PER_MINUTE * (flywheel.getVelocity() / ENCODER_TICKS_PER_REV);
    }

    public void flywheelFast() {
        setFlywheelRPM(FLYWHEEL_RPM_FAST);
    }

    public void flywheelSlow() {
        setFlywheelRPM(FLYWHEEL_RPM_SLOW);
    }

    public void flywheelStop() {
        setFlywheelRPM(0);
    }

    //antiquated
    //regression for close distance based off limelight
    public double taToDistClose(double ta){
        double scale = 77.16813;
        double exponent = -0.4654198;
        
        double dist = scale * Math.pow(ta, exponent);
        return dist;
    }
    //antiquated
    //regression for far distance based off limelight
    public double taToDistFar(double ta){
        double scale = 73.20354;
        double exponent = -0.486885;
        
        double dist = scale * Math.pow(ta, exponent);
      return dist;
    }

    //regression for pitch when close based off distance
    public double closePitch(double dist){
        double pitch = -0.00075*dist + 0.332235;
        
        return pitch;
    }

    //regression for flywheel RPM when close based off distance
    public double closeRPM(double dist){
        double rpm = 9.09090909*dist+1993.18181818;
        
        return rpm;
    }

    //regression for pitch when far based off distance
    public double farPitch(double dist){
        double pitch = 0.00921875*dist - 1.16848;
        
        return pitch;
    }

    //regression for pitch when close based off distance(not actually a regression since our data was 3400 for all parts of far)
    public double farRPM(double dist){
        double rpm = 3400;
        
        return rpm;
    }

    //changes angle from -180 to 180 into 0 to 360
    public double standardizeAngle(double angle){
        if (angle < 0){
            return angle + 360;
        }
        return angle;
    }

    //forces turret rotation when outside of bounds and stops it from rotating too far
    public double correctTurretAngle(double desiredAngle, double max, double min){
        if (desiredAngle > max && desiredAngle-360 > min){
            return desiredAngle - 360;
        }
        if (desiredAngle < min && desiredAngle+360 < max){
            return desiredAngle + 360;
        }
        return Math.min(max, (Math.max(desiredAngle, min)));
    }

    //gets the servo position from desired turret angle
    public double posFromAngle(double angle){
        return angle  / 820.0 + 0.5;
    }

    //finds the angle between the front of the robot and the target
    public double angleToTarget(double xPos, double yPos, double heading, boolean isTeamRed){
        double goalX;
        double goalY;

        if (isTeamRed) {
            goalX = 132;
            goalY = 132;
        } else {
            goalX = 12;
            goalY = 132;
        }

        double dx = goalX - xPos;
        double dy = goalY - yPos;

        double fieldTargetDeg = Math.toDegrees(Math.atan2(dy, dx)); // -180..180
        return fieldTargetDeg - heading + 90.0;
    }
    public void setPitch(double pos){
        pitch.setPosition(pos);
    }
}
