package org.firstinspires.ftc.teamcode.auton.parts;

import static org.firstinspires.ftc.teamcode.teleop.Spindexer.generateWeightArray;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.teleop.Axon;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Limelight;

import java.util.Arrays;
import java.util.List;

public class AutonSpindexer {
    public static double GEAR_RATIO = 1.5;

    private Limelight3A limelight;
    public Intake intake;

    public AnalogInput forwardEncoder;
    public AnalogInput leftEncoder;
    public AnalogInput rightEncoder;

    public CRServo crServoForward;
    public CRServo crServoLeft;
    public CRServo crServoRight;


    public Axon axonForward;
    public Axon axonLeft;
    public Axon axonRight;

    public NormalizedColorSensor spinColor;

    private double sensorAlphaSpin;
    private double trueRedSpin;
    private double trueBlueSpin;
    private double trueGreenSpin;


    public double target;
    private int obeliskId = 0;


    private boolean hasShot = false;
    private boolean hasIndexed = false;
    private boolean hasAligned = false;

    public int[] order = {2,1,1};
    public int[] correctOrder = {1,2,1};
    public double[][] weights = generateWeightArray(0.9);

    private enum Ball {
        NONE,
        PURPLE,
        GREEN
    }
    private Ball spindexerBall = Ball.NONE;


    public AutonSpindexer(HardwareMap hardwareMap, Intake intake) {
        this.intake = intake;

        forwardEncoder = hardwareMap.get(AnalogInput.class, "encoderForward");
        leftEncoder = hardwareMap.get(AnalogInput.class, "encoderLeft");
        rightEncoder = hardwareMap.get(AnalogInput.class, "encoderRight");

        crServoForward = hardwareMap.get(CRServo.class, "axonForward");
        crServoLeft = hardwareMap.get(CRServo.class, "axonLeft");
        crServoRight = hardwareMap.get(CRServo.class, "axonRight");

        axonForward = new Axon(crServoForward, forwardEncoder);
        axonLeft = new Axon(crServoLeft, leftEncoder);
        axonRight = new Axon(crServoRight, rightEncoder);

        spinColor = hardwareMap.get(NormalizedColorSensor.class, "spinColor");
        spinColor.setGain(10);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);

    }

    public Action startLimeight() {
        return packet -> {
            limelight.start();
            return false;
        };
    }

    public Action updateServos() {

        return packet -> {

            axonForward.setTargetRotation(target / GEAR_RATIO);
            axonForward.update();

            double power = axonForward.getPower();
            axonLeft.setPower(power);
            axonRight.setPower(power);

            NormalizedRGBA colorsSpin = spinColor.getNormalizedColors();

            trueRedSpin = colorsSpin.red;
            trueBlueSpin = colorsSpin.blue;
            trueGreenSpin = colorsSpin.green;
            sensorAlphaSpin = colorsSpin.alpha;

            if (ballDetectedSpin()) {
                if (ballIsGreenSpin()) {
                    spindexerBall = Ball.GREEN;
                } else if (ballIsPurpleSpin()) {
                    spindexerBall = Ball.PURPLE;
                }
            } else {
                spindexerBall = Ball.NONE;
            }

            packet.put("Spindexer Ball", spindexerBall);
            packet.put("Spindexer Target", getTargetAngle());
            packet.put("Spindexer Current", getCurrentAngle());
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

    public boolean isWithinTolerance(double current, double target) {
        return Math.abs(current-target) < 10;
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

    public Action indexBall(int color) {
        return packet -> {
            packet.addLine("INDEXING");

            if (!hasIndexed) {
                order[0] = color;
                index();
                hasIndexed = true;
            }
            if (isWithinTolerance(getCurrentAngle(), target)) {
                hasIndexed = false;
                packet.addLine("DONE INDEXING");
                return false;
            }
            return true;
        };
    }

    public Action alignForShooting() {

        return packet -> {
            packet.addLine("ALIGNING");
            if (!hasAligned) {
                align();
                target += 60;
                hasAligned = true;
            }
            if (isWithinTolerance(getCurrentAngle(), target)) {
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
                target -= 780;
                order = new int[]{0, 0, 0};
                hasShot = true;
            }
            if (isWithinTolerance(getCurrentAngle(), target)) {
                hasShot = false;
                packet.addLine("DONE SHOOTING");
                return false;
            }
            return true;

        };
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

    public void index() {
        target += 120;
        shiftArrayRight(order);
    }
    public void align() {
        int shift = findBestShift(correctOrder, order, weights);
        for (int i = 0; i < shift; i++){
            target += 120;
            shiftArrayRight(order);
        }
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
    public double getCurrentAngle() {
        return (axonForward.getTotalRotation()) * GEAR_RATIO;
    }

    public double getTargetAngle() {
        return (axonForward.getTargetRotation()) * GEAR_RATIO;
    }

    public double getSensorAlphaSpin() {return sensorAlphaSpin;}

    public double getNormalizedRedSpin(){return trueRedSpin/sensorAlphaSpin;}

    public double getNormalizedBlueSpin(){return trueBlueSpin/sensorAlphaSpin;}

    public double getNormalizedGreenSpin(){return trueGreenSpin/sensorAlphaSpin;}

}
