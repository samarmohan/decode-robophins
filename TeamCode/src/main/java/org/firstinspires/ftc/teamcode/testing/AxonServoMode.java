package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.Axon;

@TeleOp
public class AxonServoMode extends LinearOpMode {
    public AnalogInput forwardEncoder;
    public AnalogInput leftEncoder;
    public AnalogInput rightEncoder;

    public Servo servoForward;
    public Servo servoLeft;
    public Servo servoRight;


    public Axon axonForward;
    public Axon axonLeft;
    public Axon axonRight;

    public double targetAngle;
    public double targetPosition;
    public double currentAngle;
    public double currentPosition;


    @Override
    public void runOpMode() {
        forwardEncoder = hardwareMap.get(AnalogInput.class, "encoderForward");
        leftEncoder = hardwareMap.get(AnalogInput.class, "encoderLeft");
        rightEncoder = hardwareMap.get(AnalogInput.class, "encoderRight");

        servoForward = hardwareMap.get(Servo.class, "axonForward");
        servoLeft = hardwareMap.get(Servo.class, "axonLeft");
        servoRight = hardwareMap.get(Servo.class, "axonRight");

        waitForStart();

        while (opModeIsActive()) {
            targetPosition = angleToServoPosition(targetAngle);
            currentPosition = getCurrentServoPosition(forwardEncoder.getVoltage());
            currentAngle = positionToAngle(currentPosition);

            if (gamepad1.squareWasPressed()) {
                targetAngle += 60.0;
            } else if (gamepad1.triangleWasPressed()) {
                targetAngle = 0.0;
            }

            servoForward.setPosition(targetPosition);
            servoLeft.setPosition(targetPosition);
            servoRight.setPosition(targetPosition);

            telemetry.addData("Target Position", targetAngle);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addLine("------------------------------");
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.update();
        }
    }

    public double angleToServoPosition(double angle) {
        double newPosition = angle / (315 * (48.0 / 20.0));
        return Math.min(1.0, Math.max(0.0, newPosition));
    }

    public double positionToAngle(double position) {
        return position * (315 * (48.0 / 20.0));
    }

    public double servoPositionToAngle(double position) {
        return position * (315 * (48.0 / 20.0));
    }

    public double getCurrentServoPosition(double voltage) {
        return (voltage * 0.355892) - 0.0790679;
    }
}
