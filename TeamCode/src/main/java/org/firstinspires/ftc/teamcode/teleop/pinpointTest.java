package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Pinpoint Test")
@Disabled
public class pinpointTest extends OpMode {
    public GoBildaPinpointDriver pinpoint;

    @Override
    public void init(){
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            pinpoint.resetPosAndIMU();

            telemetry.addData("Status", "Pinpoint Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop(){
        pinpoint.update();  // <-- IMPORTANT: Must call update() each loop!

        telemetry.addData("X Encoder", pinpoint.getEncoderX());
        telemetry.addData("Y Encoder", pinpoint.getEncoderY());
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.update();
    }
}