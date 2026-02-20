package org.firstinspires.ftc.teamcode.testing;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.Drivetrain;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Limelight;
import org.firstinspires.ftc.teamcode.teleop.Spindexer;
import org.firstinspires.ftc.teamcode.teleop.Turret;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;

@Disabled
@TeleOp(name = "Turret Regression Finder")
public class AutoAimRegressionFinder extends LinearOpMode{
    private final Turret turret = new Turret();
    private final Drivetrain drive = new Drivetrain();
    private final Limelight limelight = new Limelight();
    private final Intake intake = new Intake();
    private final Spindexer spindexer = new Spindexer();
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        turret.init(hardwareMap);
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        limelight.init(hardwareMap);
        spindexer.init(hardwareMap, intake);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        ElapsedTime runtime = new ElapsedTime();

        HashMap<Double, Double> pitchAndRPM = new HashMap<>();

        double pitchPosition = 0.9;
        double rpm = 2800;

        limelight.start();
        limelight.setPipeline(1);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            boolean isFlywheelReady = turret.getFlywheelRPM() > 100;


            spindexer.update(
                    currentGamepad1.cross,          // In
                    currentGamepad1.right_trigger,  // Shoot
                    isFlywheelReady,
                    false,
                    false
            );

            drive.odometryUpdate();

            double heading = drive.getHeading();

            if (currentGamepad1.options) {
                drive.resetFieldCentric();
            }
            drive.driveFieldCentric(
                    -currentGamepad1.left_stick_y,
                    currentGamepad1.left_stick_x,
                    currentGamepad1.right_stick_x,
                    currentGamepad1.left_trigger
            );

            if (currentGamepad2.left_stick_y > 0.1 && pitchPosition <= 1.0) {
                pitchPosition += 0.005;
            } else if (currentGamepad2.left_stick_y < -0.1 && pitchPosition >= 0.0) {
                pitchPosition -= 0.005;
            }
            if (currentGamepad2.left_bumper) {
                rpm -= 50;
            } else if (currentGamepad2.right_bumper) {
                rpm += 50;
            }

            turret.setPitch(pitchPosition);
            turret.setTargetRPM(rpm);

            limelight.update(heading);

            boolean currentAprilTagVisible = limelight.isResultValid();

            if (currentAprilTagVisible) {

                turret.updateLimelightPID(runtime.seconds(), limelight.getTx(), true);
                turret.applyRotationPower();
            } else {
                turret.updateEncoderPID(runtime.seconds(), 0);
                turret.applyRotationPower();
            }

            turret.updateFlywheelPID(runtime.seconds());
            turret.applyFlywheelPower();

            if(gamepad2.cross){
                pitchAndRPM.put(turret.getPitch(), turret.getDistance(limelight.getTa()));
            }

            telemetry.addData("set RPM: ", rpm);
            telemetry.addData("actual RPM", turret.getFlywheelRPM());
            telemetry.addData("pitch", turret.getPitch());

            telemetry.addData("Limelight Valid?", limelight.isResultValid());
            telemetry.addData("Limelight Position", limelight.getTx());
            telemetry.addData("limelight distance", turret.getDistance(limelight.getTa()));

            telemetry.update();
        }

        String csvPath = writeToCsv(pitchAndRPM);
    }

    private String writeToCsv(HashMap<Double, Double> data) {
        String fileName = "turret_regression" + System.currentTimeMillis() + ".csv";
        File directory = new File(Environment.getExternalStorageDirectory(), "FIRST");
        if (!directory.exists()) {
            directory.mkdirs();
        }
        File file = new File(directory, fileName);

        try (FileWriter writer = new FileWriter(file)) {
            // Write header with max speed info
            writer.write("turret Pitch, distance");
            for (Double power : data.keySet()) {
                writer.write(data.get(power) + "," + power + "\n");
            }
            return file.getAbsolutePath();
        } catch (IOException e) {
            return "Error: " + e.getMessage();
        }
    }


}
