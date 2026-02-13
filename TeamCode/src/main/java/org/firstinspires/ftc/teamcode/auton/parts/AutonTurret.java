package org.firstinspires.ftc.teamcode.auton.parts;


import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.Turret;

public class AutonTurret {
    public DcMotorEx flywheel;
    public DcMotorEx flywheel2;
    public Servo pitch;

    private static final double ENCODER_TICKS_PER_REV = 28.0;
    private static final double SECONDS_PER_MINUTE = 60.0;

    public AutonTurret(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setDirection(DcMotor.Direction.FORWARD);

        flywheel.setVelocityPIDFCoefficients(300, 10, 1, 1.0);
        flywheel2.setVelocityPIDFCoefficients(300, 10, 1, 1.0);

        pitch = hardwareMap.get(Servo.class, "pitch");
        pitch.setDirection(Servo.Direction.FORWARD);
    }

        public Action setFlywheelRPM(double rpm) {
            return packet -> {
                packet.put("Flywheel Target RPM", rpm);
                flywheel.setVelocity(rpm * (ENCODER_TICKS_PER_REV / SECONDS_PER_MINUTE));
                flywheel2.setVelocity(rpm * (ENCODER_TICKS_PER_REV / SECONDS_PER_MINUTE));
                packet.put("Flywheel RPM", (flywheel.getVelocity() * 60.0) / ENCODER_TICKS_PER_REV);
                return true;
            };
        }

        public Action setPitchPosition(double pitchPos) {
            return packet -> {
                pitch.setPosition(pitchPos);
                packet.put("Pitch Position", pitchPos);
                return false;
            };
        }
}