package org.firstinspires.ftc.teamcode.auton.parts;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Spindexer;

public class AutonSpindexer extends Spindexer {
    public AutonSpindexer(HardwareMap hardwareMap, Intake intake) {
        init(hardwareMap, intake);
    }

    public Action updateServos() {
        return packet -> {
            axonForward.setTargetRotation(target / GEAR_RATIO);
            axonForward.update();

            double power = axonForward.getPower();
            axonLeft.setPower(power);
            axonRight.setPower(power);

            packet.put("Spindexer Target", getTargetAngle());
            packet.put("Spindexer Current", getCurrentAngle());

            return true;
        };
    }

    public Action startIntaking() {
        return packet -> {
            alignBack();
            intake.setState(Intake.IntakeState.INTAKE);
            hasEnteredIntaking = true;

            packet.put("Spindexer Action", "Intaking");
            return false;  // One-shot command
        };
    }

    public Action stopIntaking() {
        return packet -> {
            alignToHold();  // Close spindexer entrance
            intake.setState(Intake.IntakeState.OUTTAKE);
            hasEnteredIntaking = false;
            hasEnteredReadyToShoot = true;

            packet.put("Spindexer Action", "Stopped");
            return false;
        };
    }

    public Action indexBall() {
        return packet -> {
            // Read and identify ball color
            NormalizedRGBA colors = spinColor.getNormalizedColors();
            double red = colors.red / colors.alpha;
            double green = colors.green / colors.alpha;
            double blue = colors.blue / colors.alpha;

            if (red < 0.075 && green > 0.1 && blue > 0.075) {
                order[0] = 2;
                packet.put("Ball Color", "GREEN");
            } else if (red > 0.06 && green < 0.2 && blue > 0.1) {
                order[0] = 1;
                packet.put("Ball Color", "PURPLE");
            } else {
                order[0] = 0;
                packet.put("Ball Color", "UNKNOWN");
            }
            index();

            packet.put("Spindexer Action", "Indexed");
            packet.put("Ball Count", isFull() ? "3" : hasBalls() ? "1-2" : "0");

            return false;
        };
    }

    public Action alignForShooting() {
        return packet -> {
            if (!isFull()) {
                packet.put("Spindexer Action", "Align Skipped - Not Full");
                return false;
            }

            align();

            packet.put("Spindexer Action", "Aligned");
            packet.put("Ball Order", getOrder());

            return false;
        };
    }

    public Action shoot() {
        long startTimeNanos = System.nanoTime();

        return packet -> {
            double elapsed = (System.nanoTime() - startTimeNanos) / 1e9;

            if (elapsed < 0.1) {
                // First frame: start shooting
                alignBack();  // Undo the hold offset
                target -= 480;  // Push balls backward
                axonForward.setPower(-1);
                axonRight.setPower(-1);
                axonLeft.setPower(-1);

                packet.put("Spindexer Action", "Shooting Started");
            }

            if (elapsed >= 1.2) {
                // Shooting complete: reset everything
                axonForward.setPower(0);
                axonRight.setPower(0);
                axonLeft.setPower(0);

                setTargetAngle(getCurrentAngle());  // Lock current position
                order = new int[]{0, 0, 0};  // Clear ball tracking

                alignToHold();  // Return to hold position
                hasEnteredReadyToShoot = true;

                packet.put("Spindexer Action", "Shooting Complete");
                return false;  // Done
            }

            packet.put("Shoot Progress", String.format("%.1f / %.1f s", elapsed, 1.2));
            return true;  // Keep running
        };
    }
}
