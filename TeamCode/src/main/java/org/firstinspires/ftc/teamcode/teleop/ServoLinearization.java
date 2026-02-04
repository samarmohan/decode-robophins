package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.os.Environment;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;

@TeleOp()
public class ServoLinearization extends LinearOpMode {
    public static double GEAR_RATIO = 1.5;

    public AnalogInput forwardEncoder;
    public AnalogInput leftEncoder;
    public AnalogInput rightEncoder;

    public CRServo crServoForward;
    public CRServo crServoLeft;
    public CRServo crServoRight;

    public Axon axonForward;
    public Axon axonLeft;
    public Axon axonRight;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();

        forwardEncoder = hardwareMap.get(AnalogInput.class, "encoderForward");
        leftEncoder = hardwareMap.get(AnalogInput.class, "encoderLeft");
        rightEncoder = hardwareMap.get(AnalogInput.class, "encoderRight");

        crServoForward = hardwareMap.get(CRServo.class, "axonForward");
        crServoLeft = hardwareMap.get(CRServo.class, "axonLeft");
        crServoRight = hardwareMap.get(CRServo.class, "axonRight");

        axonForward = new Axon(crServoForward, forwardEncoder);
        axonLeft = new Axon(crServoLeft, leftEncoder);
        axonRight = new Axon(crServoRight, rightEncoder);

        double servoPower = 0.0;
        HashMap<Double, Double> powersAndSpeeds = new HashMap<>();

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && servoPower <= 1.0) {
            axonForward.setRtp(false);
            axonLeft.setRtp(false);
            axonRight.setRtp(false);

            double linearized = linearizeServo(servoPower);
            axonForward.setPower(linearized);
            axonLeft.setPower(linearized);
            axonRight.setPower(linearized);

            double lastRotation = axonForward.getTotalRotation();
            double startTime = runtime.seconds();

            while (opModeIsActive() && (runtime.seconds() - startTime) < 1.0) {
                axonForward.update();
                axonLeft.update();
                axonRight.update();

                telemetry.addData("Current Power", servoPower);
                telemetry.addData("Time until measurement", 1.0 - (runtime.seconds() - startTime));
                telemetry.addData("Current Rotation", axonForward.getTotalRotation());
                telemetry.addData("HashMap size", powersAndSpeeds.size());
                telemetry.update();
                sleep(50);
            }

            double currentRotation = axonForward.getTotalRotation();
            double rotationDelta = currentRotation - lastRotation;
            double elapsedTime = runtime.seconds() - startTime;
            double speed = rotationDelta / elapsedTime;

            powersAndSpeeds.put(servoPower, speed);

            telemetry.addData("Recorded Power", servoPower);
            telemetry.addData("Recorded Speed", speed);
            telemetry.addData("Rotation Delta", rotationDelta);
            telemetry.addData("Entries collected", powersAndSpeeds.size());
            telemetry.update();

            servoPower += 0.05;
        }


        String csvPath = writeToCsv(powersAndSpeeds);
        while (opModeIsActive()) {
            telemetry.addData("Status", "Data collection complete!");
            telemetry.addData("Total entries", powersAndSpeeds.size());
            telemetry.addData("CSV Path", csvPath);
            for (Double power : powersAndSpeeds.keySet()) {
                telemetry.addData("Power " + power, "Speed: " + powersAndSpeeds.get(power));
            }
            telemetry.update();
            idle();
        }
    }

    // Write HashMap to CSV file on robot storage
    private String writeToCsv(HashMap<Double, Double> data) {
        String fileName = "servo_linearization_" + System.currentTimeMillis() + ".csv";
        File directory = new File(Environment.getExternalStorageDirectory(), "FIRST");
        if (!directory.exists()) {
            directory.mkdirs();
        }
        File file = new File(directory, fileName);

        try (FileWriter writer = new FileWriter(file)) {
            writer.write("Power,Speed\n");
            for (Double power : data.keySet()) {
                writer.write(power + "," + data.get(power) + "\n");
            }
            return file.getAbsolutePath();
        } catch (IOException e) {
            return "Error: " + e.getMessage();
        }
    }
    private double linearizeServo(double desiredSpeed) {
        // Clamp desired speed to valid range
        desiredSpeed = Math.max(0, Math.min(450, desiredSpeed));
        double power = 0.014534 +
                2.920752e-03 * desiredSpeed +
                -1.918508e-05 * Math.pow(desiredSpeed, 2) +
                3.884573e-08 * Math.pow(desiredSpeed, 3);

        // Clamp output to valid range [0, 1]
        return Math.max(0, Math.min(1, power));
    }
}
