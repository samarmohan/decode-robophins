package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

public class ServoSpindexer {
    public static double GEAR_RATIO = 1.5;

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

    public int isWithinTolerance;

    private boolean hasShot;
    private boolean hasIndexed;
    private boolean hasAligned;

    public int[] order = {0,0,0};
    public int[] correctOrder = {2,1,1};
    public double[][] weights = generateWeightArray(0.9);

    private ElapsedTime indexTimer;
    private ElapsedTime alignTimer;
    private ElapsedTime shootTimer;
    private ElapsedTime intakeTimer;

    public enum SpindexerState {
        INTAKING,
        INDEXING,
        ALIGNING,
        READY_TO_SHOOT,
        SHOOTING,

    }

    private SpindexerState spindexerState = SpindexerState.READY_TO_SHOOT;
    private Intake.IntakeState intakeState = Intake.IntakeState.OFF;

    public void init(HardwareMap hardwareMap, Intake intake) {
        //allows for controlling intake state
        this.intake = intake;

        //axon encoders
        forwardEncoder = hardwareMap.get(AnalogInput.class, "encoderForward");
        leftEncoder = hardwareMap.get(AnalogInput.class, "encoderLeft");
        rightEncoder = hardwareMap.get(AnalogInput.class, "encoderRight");

        //axon servos
        forwardServo = hardwareMap.get(Servo.class, "axonForward");
        leftServo = hardwareMap.get(Servo.class, "axonLeft");
        rightServo = hardwareMap.get(Servo.class, "axonRight");

        //color sensor
        spinColor = hardwareMap.get(NormalizedColorSensor.class, "spinColor");
        spinColor.setGain(10);

        //timers
        indexTimer = new ElapsedTime();
        indexTimer.reset();
        alignTimer = new ElapsedTime();
        alignTimer.reset();
        shootTimer = new ElapsedTime();
        shootTimer.reset();
        intakeTimer = new ElapsedTime();
        intakeTimer.reset();

        //default servo position
        targetAngle = 240;
        targetPosition = angleToPosition(targetAngle);
        currentAngle = positionToAngle(getCurrentPosition(getVoltage()));
        setServoPositions(targetPosition);
    }

    /**
     * Main update loop - handles all spindexer logic and state transitions
     *
     * @param inButton - true when driver holds the "in" button
     * @param shootTrigger - value of shoot trigger (>0.1 = shooting)
     * @param isFlywheelReady - true when flywheel is at speed and ready to shoot
     */
    public void update(boolean inButton, double shootTrigger, boolean isFlywheelReady, boolean indexOverride, boolean shouldSort) {

        targetPosition = angleToPosition(targetAngle);

        currentPosition = getCurrentPosition(getVoltage());
        currentAngle = positionToAngle(currentPosition);

        NormalizedRGBA colorsSpin = spinColor.getNormalizedColors();

        trueRedSpin = colorsSpin.red;
        trueBlueSpin = colorsSpin.blue;
        trueGreenSpin = colorsSpin.green;
        sensorAlphaSpin = colorsSpin.alpha;

        switch (spindexerState) {
            case INTAKING:
                //set intake
                intakeState = Intake.IntakeState.INTAKE;
                //ball detection for indexing
                if (ballDetectedSpin() && intakeTimer.seconds() > 0.5 || indexOverride) {
                    if (ballIsGreenSpin()) {
                        order[0] = 2;
                    } else {
                        order[0] = 1;
                    }
                    //switching state to indexing
                    hasIndexed = false;
                    indexTimer.reset();
                    spindexerState = SpindexerState.INDEXING;
                }
                if (!inButton) {
                    //moves out of intaking if stop pressing to intake
                    alignToHold();
                    spindexerState = SpindexerState.READY_TO_SHOOT;
                }
                break;
            case INDEXING:
                //set intake state
                intakeState = Intake.IntakeState.INTAKE;
                //first loop indexes
                if (!hasIndexed) {
                    hasIndexed = true;
                    index();
                }
                //once indexing is done
                if (isWithinTolerance(currentAngle, targetAngle) || indexTimer.seconds() > 1) {
                    if (isFull() && shouldSort) {
                        //if full starts to align
                        alignTimer.reset();
                        hasAligned = false;
                        spindexerState = SpindexerState.ALIGNING;
                    } else if (inButton) {
                        //if holding in, goes back to intaking
                        intakeTimer.reset();
                        spindexerState = SpindexerState.INTAKING;
                    } else {
                        //defaults to hold position
                        alignToHold();
                        spindexerState = SpindexerState.READY_TO_SHOOT;
                    }
                }
                break;
            case ALIGNING:
                //set intake state
                intakeState = Intake.IntakeState.OUTTAKE;
                //first loop runs align
                if (!hasAligned) {
                    align();
                    hasAligned = true;
                }
                //once done switches to ready to shoot
                if (isWithinTolerance(currentAngle, targetAngle) || alignTimer.seconds() > 1) {
                    alignToHold();
                    spindexerState = SpindexerState.READY_TO_SHOOT;
                }
                break;
            case READY_TO_SHOOT:
                //sets intake state
                intakeState = Intake.IntakeState.OUTTAKE;
                //if there is space allows you to intake
                if (inButton && !isFull()) {
                    alignBack();
                    intakeTimer.reset();
                    spindexerState = SpindexerState.INTAKING;
                }
                //if flywheel is up to RPM allows you to shoot
                else if (shootTrigger > 0.2 && isFlywheelReady) {
                    shootTimer.reset();
                    hasShot = false;
                    spindexerState = SpindexerState.SHOOTING;
                }
                break;
            case SHOOTING:
                //sets intake state
                intakeState = Intake.IntakeState.OUTTAKE;
                //first loop runs shoot
                if (!hasShot) {
                    shoot();
                    hasShot = true;
                }
                //once done goes back to ready to shoot(defualt state)
                if (isWithinTolerance(currentAngle, targetAngle) || shootTimer.seconds() > 1) {
                    hasShot = false;
                    spindexerState = SpindexerState.READY_TO_SHOOT;
                }
                break;
        }
        intake.setState(intakeState);
        setServoPositions(targetPosition);
    }

    public Intake.IntakeState getIntakeCommand() {
        return intakeState;
    }

    public void setServoPositions(double position) {
        forwardServo.setPosition(position);
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double angleToPosition(double angle) {
        double newPosition = angle / (315 * (48.0 / 20.0));
        return Math.min(1.0, Math.max(0.0, newPosition));
    }

    public double positionToAngle(double position) {
        return position * (315 * (48.0 / 20.0));
    }

    public double getVoltage() {
        return forwardEncoder.getVoltage();
    }

    public double getCurrentPosition(double voltage) {
        return (voltage * 0.355892) - 0.0790679;
    }

    public boolean isWithinTolerance(double current, double target) {
        return Math.abs(current-target) < 10;
    }
    public void index() {
        targetAngle += 120;
        shiftArrayRight(order);
    }
    public void shoot() {
        targetAngle -= 720;
        order = new int[]{0, 0, 0};
    }
    public void align() {
        int shift = findBestShift(correctOrder, order, weights);
        for (int i = 0; i < shift; i++){
            targetAngle += 120;
            shiftArrayRight(order);
        }
    }
    public void alignBack() {
        targetAngle -= 60;
    }
    public void alignToHold() {
        targetAngle += 60;
    }

    public void resetTarget() {
        targetAngle = 0;
    }
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

    public boolean ballDetectedSpin(){
        isWithinTolerance = Math.floorMod((int)currentAngle, 120);
        //value not tuned
        if ((isWithinTolerance > 110 || isWithinTolerance < 10) && sensorAlphaSpin > 0.2) {
            return true;
        }
        return false;
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
        double[][] array = {
                {hit, 0.0, 0.0},
                {miss*hit,hit*hit,0.0},
                {miss*miss*hit, miss*hit*hit + hit*miss*hit, hit*hit*hit}
        };
        return array;
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


    public double getSensorAlphaSpin() {return sensorAlphaSpin;}

    public double getNormalizedRedSpin(){return trueRedSpin/sensorAlphaSpin;}

    public double getNormalizedBlueSpin(){return trueBlueSpin/sensorAlphaSpin;}

    public double getNormalizedGreenSpin(){return trueGreenSpin/sensorAlphaSpin;}

    public String getOrder(){
        return Arrays.toString(order);
    }
    public SpindexerState getState() {
        return spindexerState;
    }

    public void setState(SpindexerState spindexerState) {
        this.spindexerState = spindexerState;
    }

    public void setCorrectOrder(int[] correctOrder) {
        this.correctOrder = correctOrder;
    }

}
