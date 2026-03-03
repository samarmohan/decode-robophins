package org.firstinspires.ftc.teamcode.auton.parts;

import static org.firstinspires.ftc.teamcode.teleop.Spindexer.generateWeightArray;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.Intake;

import java.util.Arrays;
import java.util.List;

public class AutonSpindexer {
    public static double GEAR_RATIO = 48.0 / 20.0;
    private Limelight3A limelight;
    public Intake intake;

    public double targetAngle;
    public double targetPosition;
    public double currentAngle;
    public double currentPosition;

    public AnalogInput forwardEncoder;
    public AnalogInput leftEncoder;
    public AnalogInput rightEncoder;

    public Servo forwardServo;
    public Servo leftServo;
    public Servo rightServo;


    public NormalizedColorSensor spinColor;

    private double sensorAlphaSpin;
    private double trueRedSpin;
    private double trueBlueSpin;
    private double trueGreenSpin;

    private int obeliskId = 0;


    private boolean hasShot = false;
    private boolean hasIndexed = false;
    private boolean hasAligned = false;

    public int[] order = {2,1,1};
    public int[] correctOrder = {1,2,1};
    public double[][] weights = generateWeightArray(0.9);

    private ElapsedTime shootTimer;

    private final double CUTOFF_DISTANCE = 7.0;

    public AutonSpindexer(HardwareMap hardwareMap, Intake intake) {
        this.intake = intake;

        //axon encoders
        forwardEncoder = hardwareMap.get(AnalogInput.class, "encoderForward");
        leftEncoder = hardwareMap.get(AnalogInput.class, "encoderLeft");
        rightEncoder = hardwareMap.get(AnalogInput.class, "encoderRight");

        //axon servos
        forwardServo = hardwareMap.get(Servo.class, "axonForward");
        leftServo = hardwareMap.get(Servo.class, "axonLeft");
        rightServo = hardwareMap.get(Servo.class, "axonRight");

        spinColor = hardwareMap.get(NormalizedColorSensor.class, "spinColor");
        spinColor.setGain(10);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);

        shootTimer = new ElapsedTime();
        shootTimer.reset();

        //default servo position
        setTargetAngle(240);

        currentPosition = getCurrentPosition(getVoltage());
        currentAngle = positionToAngle(currentPosition);

    }

    public Action startLimeight() {
        return packet -> {
            limelight.start();
            return false;
        };
    }

    public Action updateSpindexer() {

        return packet -> {
            currentPosition = getCurrentPosition(getVoltage());
            currentAngle = positionToAngle(currentPosition);

            packet.put("Spindexer Target", getTargetAngle());
            packet.put("Spindexer Current", currentAngle);
            packet.put("Order", Arrays.toString(order));
            packet.put("ID", obeliskId);

            return true;
        };
    }

    public Action getObelisk() {
        return packet -> {
            packet.addLine("GETTING OBELISK");
            List<LLResultTypes.FiducialResult> result = limelight.getLatestResult().getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult : result) {
                obeliskId = fiducialResult.getFiducialId();
            }
            if (obeliskId == 0) {
                return true;
            }
            if (obeliskId == 21){
                correctOrder = new int[] {2, 1, 1};
            }
            if (obeliskId == 22){
                correctOrder = new int[] {1, 2, 1};
            }
            if (obeliskId == 23){
                correctOrder = new int[] {1, 1, 2};
            }
            return false;
        };
    }

    public Action off() {
        return packet -> {
          intake.setState(Intake.IntakeState.OFF);
          return false;
        };
    }

    public Action intake() {
        return packet -> {
            packet.addLine("INTAKING");
            intake.setState(Intake.IntakeState.INTAKE);
            return false;
        };
    }

    public Action outtake() {
        return packet -> {
            packet.addLine("STOPPED");
            intake.setState(Intake.IntakeState.OUTTAKE);
            return false;
        };
    }

    public Action index(int color) {
        return packet -> {
            packet.addLine("INDEXING");

            if (!hasIndexed) {
                order[0] = color;
                setIndex();
                hasIndexed = true;
            }
            if (isWithinTolerance(currentAngle, targetAngle)) {
                hasIndexed = false;
                packet.addLine("DONE INDEXING");
                return false;
            }
            return true;
        };
    }

    public Action align() {
        return packet -> {
            packet.addLine("ALIGNING");
            if (!hasAligned) {
                setAlign();
                alignToHold();
                hasAligned = true;
            }
            if (isWithinTolerance(currentAngle, targetAngle)) {
                hasAligned = false;
                packet.addLine("DONE ALIGNING");
                return false;
            }
            return true;
        };
    }

    public Action shoot() {
        return packet -> {
            packet.addLine("SHOOTING");

            if (!hasShot) {
                setShoot();
                hasShot = true;
            }
            if (isWithinTolerance(currentAngle, targetAngle)) {
                hasShot = false;
                packet.addLine("DONE SHOOTING");
                return false;
            }
            return true;

        };
    }

    public void setIndex() {
        setTargetAngle(targetAngle + 120);
        shiftArrayRight(order);
    }
    public void setShoot() {
        setTargetAngle(0);
        order = new int[]{0, 0, 0};
    }
    public void setAlign() {
        int shift = findBestShift(correctOrder, order, weights);
        for (int i = 0; i < shift; i++){
            setTargetAngle(targetAngle + 120);
            shiftArrayRight(order);
        }
    }
    public void alignBack() {
        setTargetAngle(targetAngle - 60);
    }
    public void alignToHold() {
        setTargetAngle(targetAngle + 60);
    }
    public void alignToStart(){
        setTargetAngle(240);
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
        targetPosition = angleToPosition((Math.min(Math.max(targetAngle, 0),720)));
        forwardServo.setPosition(targetPosition);
        leftServo.setPosition(targetPosition);
        rightServo.setPosition(targetPosition);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double angleToPosition(double angle) {
        double newPosition = angle / (315 * (GEAR_RATIO));
        return Math.min(1.0, Math.max(0.0, newPosition));
    }

    public double positionToAngle(double position) {
        return position * (315 * (GEAR_RATIO));
    }

    public double getVoltage() {
        return forwardEncoder.getVoltage();
    }

    public double getVoltageAverage(){
        return (getVoltage() + rightEncoder.getVoltage())/2;
    }

    public double getCurrentPosition(double voltage) {
        return (0.351385 * voltage) - 0.076737;
    }

    public boolean isWithinTolerance(double current, double target) {
        return Math.abs(current-target) < 10;
    }

    public static void shiftArrayRight(int[] array) {
        int lastIndex = array.length - 1;
        int oldLast = array[lastIndex];
        for (int i = lastIndex; i > 0; i--) {
            array[i] = array[i - 1];
        }
        array[0] = oldLast;
    }

    public static int[] removeEmpty(int[] balls){
        int[] newBalls = {0,0,0};
        int i=0;
        for(int ball : balls){
            if (ball != 0){
                newBalls[i++] = ball;
            }
        }
        return newBalls;
    }


    public static int findBestShift(int[] order, int[] balls, double[][] weights){
        int bestShift = 0;
        double bestValue = 0;
        for (int i = 0; i < 3; i++){
            double currentValue = findWeightedValue(order, removeEmpty(balls), weights);
            if(currentValue > bestValue){
                bestShift = i;
                bestValue = currentValue;
            }
            shiftArrayRight(balls);
        }
        return bestShift;
    }

    public static double findBallValue(int[] order, int ball, double[] weight){
        double value = 0;
        for (int i = 0; i < 3; i++){
            if (ball == order[i]){
                value += weight[i];
            }
        }
        return value;
    }

    //adds all values for 3 balls in a specific shift
    public static double findWeightedValue(int[] order, int[] balls, double[][] weights){
        double total = 0;
        for (int i = 0; i < 3; i++){
            total += findBallValue(order, balls[i], weights[i]);
        }
        return total;
    }


    public boolean ballDetectedSpin(){
        //value not tuned
        return sensorAlphaSpin > 0.4;
    }

    public boolean ballIsGreenSpin(){
        //not tuned
        return ballDetectedSpin() &&
                getNormalizedRedSpin() < 0.075 &&
                getNormalizedGreenSpin() > 0.1 &&
                getNormalizedBlueSpin() > 0.075;
    }

    public boolean ballIsPurpleSpin(){
        //not tuned
        return ballDetectedSpin() &&
                getNormalizedRedSpin() > 0.06 &&
                getNormalizedGreenSpin() < 0.2 &&
                getNormalizedBlueSpin() > 0.1;
    }

    public double getSensorAlphaSpin() {return sensorAlphaSpin;}

    public double getNormalizedRedSpin(){return trueRedSpin/sensorAlphaSpin;}

    public double getNormalizedBlueSpin(){return trueBlueSpin/sensorAlphaSpin;}

    public double getNormalizedGreenSpin(){return trueGreenSpin/sensorAlphaSpin;}

}
