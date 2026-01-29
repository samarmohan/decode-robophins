package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Tilt {
        public Servo tilt;

        private final double UP_POS = 0.4;
        private final double DOWN_POS = 0.63;

        public void init(HardwareMap hardwareMap){
            tilt = hardwareMap.get(Servo.class, "tilt");

            tilt.setDirection(Servo.Direction.REVERSE);
        }

        public void moveUp(){
            tilt.setPosition(UP_POS);
        }

        public void moveDown(){
            tilt.setPosition(DOWN_POS);
        }

}
