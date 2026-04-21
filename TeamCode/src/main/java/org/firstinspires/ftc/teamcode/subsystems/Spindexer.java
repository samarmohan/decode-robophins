package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.TeleOpConstants.*;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.States;

import java.util.Arrays;

public class Spindexer {
    //--- Hardware ---
    public AnalogInput forwardEncoder;
    public AnalogInput leftEncoder;
    public AnalogInput rightEncoder;
    public Servo forwardServo;
    public Servo leftServo;
    public Servo rightServo;
    public RevColorSensorV3 spinColor;
    public RevColorSensorV3 spinColor2;
    public Rev2mDistanceSensor backColor;
    //--- Movement Variables ---
    public double targetAngle = 240;
    public double targetPosition;
    public double currentAngle;
    public double currentPosition;
    //--- Ordering Variables ---
    public int[] order = {0,0,0};
    public int[] correctOrder = {2,1,1};
    public double[][] weights = generateWeightArray(0.9);
    //--- Sensor Variables ---
    private final double CUTOFF_DISTANCE = 7.0;//back sensor cutoff
    //--- Offset Override Variables ---
    private double OFFSET_ANGLE = 0.0;
    private double offsetTargetAngle = 0.0;
    //--- State Machine Variables ---
    private boolean shouldSort = false;
    private boolean hasShot = false;
    private boolean isShooting = false;
    private ElapsedTime shootTimer;
    public enum SpindexerState {
        INTAKING,
        INDEXING,
        ALIGNING,
        READY_TO_SHOOT,
        SHOOTING,

    }
    public States.IntakeState intakeState = States.IntakeState.OFF;
    private SpindexerState spindexerState = SpindexerState.READY_TO_SHOOT;
    //--- Constructor
    public Spindexer(HardwareMap hardwareMap){
        //axon encoders
        forwardEncoder = hardwareMap.get(AnalogInput.class, "encoderForward");
        leftEncoder = hardwareMap.get(AnalogInput.class, "encoderLeft");
        rightEncoder = hardwareMap.get(AnalogInput.class, "encoderRight");

        //axon servos
        forwardServo = hardwareMap.get(Servo.class, "axonForward");
        leftServo = hardwareMap.get(Servo.class, "axonLeft");
        rightServo = hardwareMap.get(Servo.class, "axonRight");

        //color sensor
        spinColor = hardwareMap.get(RevColorSensorV3.class, "spinColor");
        spinColor.setGain(10);

        spinColor2 = hardwareMap.get(RevColorSensorV3.class, "spinColor2");
        spinColor2.setGain(10);

        backColor = hardwareMap.get(Rev2mDistanceSensor.class, "backColor");

        shootTimer = new ElapsedTime();
        shootTimer.reset();

        setTargetAngle(240);
        currentPosition = getCurrentPosition(getVoltage());
        currentAngle = positionToAngle(currentPosition);
    }
    //--- Main Loop Function ---
    public void update(boolean inButton, boolean shoot, boolean isFlywheelReady, boolean indexOverride) {
        currentPosition = getCurrentPosition(getVoltage());
        currentAngle = positionToAngle(currentPosition);

        switch(spindexerState){
            case INTAKING:
                intakeState = States.IntakeState.INTAKE;
                if ((isWithinTolerance(currentAngle, targetAngle) && ballDetectedSpin()) || indexOverride) {
                    if (ballIsGreenSpin()) {
                        order[0] = 2;
                    } else {
                        order[0] = 1;
                    }
                    //switching state to indexing
                    if(!isFull()) {
                        spindexerState = SpindexerState.INDEXING;
                    } else if (shouldSort) {
                        //if full starts to align
                        spindexerState = SpindexerState.ALIGNING;
                    } else {
                        alignToHold();
                        spindexerState = SpindexerState.READY_TO_SHOOT;
                    }
                }
                if (!inButton) {
                    //moves out of intaking if stop pressing to intake
                    alignToHold();
                    spindexerState = SpindexerState.READY_TO_SHOOT;
                }
                break;
            case INDEXING:
                intakeState = States.IntakeState.OFF;
                index();
                //if (isWithinTolerance(currentAngle, targetAngle)) {
                    if (inButton) {
                        //if holding in, goes back to intaking
                        spindexerState = SpindexerState.INTAKING;
                    } else {
                        //defaults to hold position
                        alignToHold();
                        spindexerState = SpindexerState.READY_TO_SHOOT;
                    }
                 //}
                break;
            case ALIGNING:
                intakeState = States.IntakeState.OUTTAKE;
                align();
                alignToHold();
                spindexerState = SpindexerState.READY_TO_SHOOT;
                break;
            case READY_TO_SHOOT:
                intakeState = States.IntakeState.OUTTAKE;
                //if there is space allows you to intake
                if (inButton && !isFull()) {
                    alignBack();
                    spindexerState = SpindexerState.INTAKING;
                }
                //if flywheel is up to RPM allows you to shoot
                // TODO add isFlywheelReady
                else if (shoot /*&& isFlywheelReady*/) {
                    shootTimer.reset();
                    hasShot = false;
                    spindexerState = SpindexerState.SHOOTING;
                }
                break;
            case SHOOTING:
                intakeState = States.IntakeState.OUTTAKE;
                //first loop runs shoot
                if (!hasShot) {
                    shoot();
                    hasShot = true;
                    isShooting = true;
                }
                //once done goes back to ready to shoot(defualt state)
                if (isWithinTolerance(currentAngle, targetAngle) || shootTimer.seconds() > 3) {
                    alignToStart();
                    isShooting = false;
                    spindexerState = SpindexerState.READY_TO_SHOOT;
                }
                break;
        }
    }
    //--- Movement Helpers ---
    public void setTargetAngle(double angle) {
        targetAngle = angle;
        targetPosition = angleToPosition((Math.min(Math.max(targetAngle, 0),720)));

        double OFFSET_POSITION = angleToPosition(Math.min(30, Math.max(OFFSET_ANGLE, 0)));
        forwardServo.setPosition(targetPosition+OFFSET_POSITION);
        leftServo.setPosition(targetPosition+OFFSET_POSITION);
        rightServo.setPosition(targetPosition+OFFSET_POSITION);
    }
    public double getTargetAngle() {
        return targetAngle;
    }
    public double angleToPosition(double angle) {
        double newPosition = angle / (315 * (SPINDEXER_GEAR_RATIO));
        return Math.min(1.0, Math.max(0.0, newPosition));
    }

    public double positionToAngle(double position) {
        return position * (315 * (SPINDEXER_GEAR_RATIO));
    }

    public double getVoltage() {
        return forwardEncoder.getVoltage();
    }
    public double getCurrentPosition(double voltage) {
        return (0.351385 * voltage) - 0.076737;
    }

    public boolean isWithinTolerance(double current, double target) {
        return Math.abs(current-target) < 10;
    }
    public void index() {
        setTargetAngle(targetAngle + 120);
        shiftArrayRight(order);
    }
    public void shoot() {
        setTargetAngle(0);
        order = new int[]{0, 0, 0};
    }
    public void align() {
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
    //--- Ordering Helpers ---
    public boolean hasBalls() {
        for (int i : order) {
            if (i != 0) {
                return true;
            }
        }
        return false;
    }
    public boolean isFull(){
        for (int i : order){
            if (i == 0){
                return false;
            }
        }
        return true;
    }
    //array stuff
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

    //finds the shift with the highest value (if two are the same it chooses the lower shift value)
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

    //puts 0 value balls(empty slots) at the back since they dont actually get shot (i.e. if you shoot green-empty-green, it really just shoots green-green)
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
    //creates a array of weights bassed of the probability that the ball goes into each slot(currently only uses percent we make, later can implement probability of one going in before the other)
    public static double[][] generateWeightArray(double hit){
        double miss = 1 - hit;
        return new double[][]{
                {hit, 0.0, 0.0},
                {miss*hit,hit*hit,0.0},
                {miss*miss*hit, miss*hit*hit + hit*miss*hit, hit*hit*hit}
        };
    }
    //shifts array 1 index to the left ([a,b,c] -> [b,c,a])
    public static void shiftArrayLeft(int[] array) {
        int lastIndex = array.length - 1;
        int oldFirst = array[0];
        for (int i = 0; i < lastIndex; i++) {
            array[i] = array[i + 1];
        }
        array[lastIndex] = oldFirst;
    }
    //shifts array 1 index to the right ([a,b,c]->[c,a,b])
    public static void shiftArrayRight(int[] array) {
        int lastIndex = array.length - 1;
        int oldLast = array[lastIndex];
        for (int i = lastIndex; i > 0; i--) {
            array[i] = array[i - 1];
        }
        array[0] = oldLast;
    }
    //--- Sensor Helpers ---
    public boolean ballDetectedSpin(){
        return (getBackDistance() < CUTOFF_DISTANCE) && !(getSpinDistance() <0.9);
    }

    public boolean ballIsGreenSpin(){
        return ballDetectedSpin() &&
                getNormalizedRedSpin() < 0.075 &&
                getNormalizedGreenSpin() > 0.1 &&
                getNormalizedBlueSpin() > 0.075;
    }
    public double getSensorAlphaSpin() {return spinColor.getNormalizedColors().alpha;}
    public double getSensorRedSpin() {return spinColor.getNormalizedColors().red;}
    public double getSensorBlueSpin() {return spinColor.getNormalizedColors().blue;}
    public double getSensorGreenSpin() {return spinColor.getNormalizedColors().green;}

    public double getNormalizedRedSpin() {
        return (getSensorAlphaSpin() > 0.001) ? (getSensorRedSpin() / getSensorAlphaSpin()) : 0.0;
    }
    public double getNormalizedBlueSpin() {
        return (getSensorAlphaSpin() > 0.001) ? (getSensorBlueSpin() / getSensorAlphaSpin()) : 0.0;
    }
    public double getNormalizedGreenSpin() {
        return (getSensorAlphaSpin() > 0.001) ? (getSensorGreenSpin() / getSensorAlphaSpin()) : 0.0;
    }
    //--- Getters and Setters ---
    public String getOrder(){
        return Arrays.toString(order);
    }
    public SpindexerState getState() {
        return spindexerState;
    }
    public double getCurrentAngle(){
        return currentAngle;
    }
    public double getBackDistance(){
        return backColor.getDistance(DistanceUnit.CM);
    }
    public double getSpinDistance(){
        return spinColor.getDistance(DistanceUnit.CM);
    }
    public void setSorting(boolean sorting){
        shouldSort = sorting;
    }
    public boolean getShouldSort(){
        return shouldSort;
    }
    public boolean shouldIntake(){
        return spindexerState == SpindexerState.INDEXING || spindexerState == SpindexerState.INTAKING;
    }
    public boolean isShooting(){
        return isShooting;
    }
    public States.IntakeState getIntakeState(){
        return intakeState;
    }
}
