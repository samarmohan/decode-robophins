package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
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

        double servoPower = 0.05; // Start at 0.05 to avoid dead zone
        HashMap<Double, Double> powersAndSpeeds = new HashMap<>();
        double maxSpeed = 0; // Track maximum speed for normalization

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && servoPower <= 1.0) {
            axonForward.setRtp(false);
            axonLeft.setRtp(false);
            axonRight.setRtp(false);

            // Apply RAW power (no linearization during data collection!)
            axonForward.setPower(servoPower);
            axonLeft.setPower(servoPower);
            axonRight.setPower(servoPower);

            // Wait for servo to reach steady-state speed
            double settleStartTime = runtime.seconds();
            while (opModeIsActive() && (runtime.seconds() - settleStartTime) < 0.5) {
                axonForward.update();
                axonLeft.update();
                axonRight.update();

                telemetry.addData("Current Power", servoPower);
                telemetry.addData("Settling...", 0.5 - (runtime.seconds() - settleStartTime));
                telemetry.update();
                sleep(50);
            }

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

            // Track maximum speed
            if (speed > maxSpeed) {
                maxSpeed = speed;
            }

            powersAndSpeeds.put(servoPower, speed);

            telemetry.addData("Recorded Power", servoPower);
            telemetry.addData("Recorded Speed", speed);
            telemetry.addData("Rotation Delta", rotationDelta);
            telemetry.addData("Entries collected", powersAndSpeeds.size());
            telemetry.update();

            servoPower += 0.05;
        }

        // Normalize all speeds to 0-1 range
        HashMap<Double, Double> normalizedData = new HashMap<>();
        for (Double power : powersAndSpeeds.keySet()) {
            double rawSpeed = powersAndSpeeds.get(power);
            double normalizedSpeed = rawSpeed / maxSpeed;
            normalizedData.put(power, normalizedSpeed);
        }

        String csvPath = writeToCsv(normalizedData, maxSpeed);
        while (opModeIsActive()) {
            telemetry.addData("Status", "Data collection complete!");
            telemetry.addData("Total entries", normalizedData.size());
            telemetry.addData("Max speed measured", String.format("%.2f deg/sec", maxSpeed));
            telemetry.addData("CSV Path", csvPath);
            telemetry.addLine();
            telemetry.addLine("Power -> Normalized Speed (0-1):");
            for (Double power : normalizedData.keySet()) {
                telemetry.addData(String.format("%.2f", power), String.format("%.3f", normalizedData.get(power)));
            }
            telemetry.update();
            idle();
        }
    }

    // Write HashMap to CSV file on robot storage
    private String writeToCsv(HashMap<Double, Double> data, double maxSpeed) {
        String fileName = "servo_linearization_" + System.currentTimeMillis() + ".csv";
        File directory = new File(Environment.getExternalStorageDirectory(), "FIRST");
        if (!directory.exists()) {
            directory.mkdirs();
        }
        File file = new File(directory, fileName);

        try (FileWriter writer = new FileWriter(file)) {
            // Write header with max speed info
            writer.write("# Max Speed: " + maxSpeed + " deg/sec\n");
            writer.write("Normalized_Speed,Power\n");
            for (Double power : data.keySet()) {
                writer.write(data.get(power) + "," + power + "\n");
            }
            return file.getAbsolutePath();
        } catch (IOException e) {
            return "Error: " + e.getMessage();
        }
    }
}