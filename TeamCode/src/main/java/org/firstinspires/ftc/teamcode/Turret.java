package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


public class Turret {
    public DcMotorEx flywheel;
    public DcMotorEx flywheel2;
    public CRServo rightR;
    public CRServo leftR;
    public Servo pitch;

    private static final double ENCODER_TICKS_PER_REV = 28.0;
    private static final double SECONDS_PER_MINUTE = 60.0;

    public static final double FLYWHEEL_RPM_FAST = 3400.0;
    public static final double FLYWHEEL_RPM_SLOW = 2800.0;



    public void init(HardwareMap hardwareMap) {
        //rotation = hardwareMap.get(DcMotorEx.class, "rotation");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

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

        pitch = hardwareMap.get(Servo.class, "pitch");
        rightR = hardwareMap.get(CRServo.class, "rightR");
        leftR  = hardwareMap.get(CRServo.class, "leftR");
        pitch.setDirection(Servo.Direction.REVERSE);
        rightR.setDirection(CRServo.Direction.FORWARD);
        leftR.setDirection(CRServo.Direction.FORWARD);
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

    //regression for pitch when close based off distance
    public double closePitch(double dist){
        return -0.00075*dist + 0.332235;
    }

    //regression for flywheel RPM when close based off distance
    public double closeRPM(double dist){

        return 9.09090909*dist+1993.18181818;
    }

    //regression for pitch when far based off distance
    public double farPitch(double dist){

        return 0.00921875*dist - 1.16848;
    }

    //regression for pitch when close based off distance(not actually a regression since our data was 3400 for all parts of far)
    public double farRPM(double dist){

        return 3400;
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
        return Math.max(0.0, Math.min(1.0, angle  / 820.0 + 0.5));
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

    public void setRotationPower(double power){
        rightR.setPower(power);
        leftR.setPower(power);
    }

    //the through bore encoder is plugged into flywheel 2 slot
    public double getRotationRealPosition(){
        return flywheel2.getCurrentPosition();
    }

    public void rotationPID(double targetAngle){

    }
}
