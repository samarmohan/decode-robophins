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


    @Override
    public void runOpMode() {
        forwardEncoder = hardwareMap.get(AnalogInput.class, "encoderForward");
        leftEncoder = hardwareMap.get(AnalogInput.class, "encoderLeft");
        rightEncoder = hardwareMap.get(AnalogInput.class, "encoderRight");

        servoForward = hardwareMap.get(Servo.class, "axonForward");
        servoLeft = hardwareMap.get(Servo.class, "axonLeft");
        servoRight = hardwareMap.get(Servo.class, "axonRight");

        double angle = 0.0;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.squareWasPressed()) {
                angle += 90.0;
            } else if (gamepad1.triangleWasPressed()) {
                angle = 0.0;
            }

            double servoPosition = angleToServoPosition(angle);
            servoForward.setPosition(servoPosition);
            servoLeft.setPosition(servoPosition);
            servoRight.setPosition(servoPosition);

            telemetry.addData("Target Position", servoForward.getPosition());
            telemetry.addData("Current Position", getCurrentServoPosition(forwardEncoder.getVoltage()));
            telemetry.addLine("------------------------------");
            telemetry.addData("Target Angle", servoPositionToAngle(servoForward.getPosition()));
            telemetry.addData("Current Angle", servoPositionToAngle(getCurrentServoPosition(forwardEncoder.getVoltage())));
            telemetry.update();
        }
    }

    public double angleToServoPosition(double angle) {
        double newPosition = angle / (315 * (48.0 / 20.0));
        return Math.min(1.0, Math.max(0.0, newPosition));
    }

    public double servoPositionToAngle(double position) {
        return position * (315 * (48.0 / 20.0));
    }

    public double getCurrentServoPosition(double voltage) {
        return (voltage * 0.355892) - 0.0790679;
    }
}
