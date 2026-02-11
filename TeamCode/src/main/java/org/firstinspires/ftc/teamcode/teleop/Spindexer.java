package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

public class Spindexer {
    public static double GEAR_RATIO = 1.5;

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

    // DEPRECATED INTAKE SENSOR
    private double sensorAlphaIntake;
    private double trueRedIntake;
    private double trueBlueIntake;
    private double trueGreenIntake;
    private boolean ballInIntake;


    private double sensorAlphaSpin;
    private double trueRedSpin;
    private double trueBlueSpin;
    private double trueGreenSpin;


    public double target;
    private boolean powerOverride = false;


    private boolean hasShot;
    private boolean hasIndexed;
    private boolean hasAligned;

    public boolean hasEnteredIntaking = false;
    public boolean hasEnteredReadyToShoot = false;

    public int[] order = {0,0,0};
    public int[] correctOrder = {2,1,1};
    public double[][] weights = generateWeightArray(0.9);

    private ElapsedTime indexTimer;
    private ElapsedTime alignTimer;
    private ElapsedTime shootTimer;

    public enum SpindexerState {
        INTAKING,
        INDEXING,
        ALIGNING,
        READY_TO_SHOOT,
        SHOOTING,

    }

    private enum Ball {
        NONE,
        PURPLE,
        GREEN
    }
    private SpindexerState spindexerState = SpindexerState.READY_TO_SHOOT;
    private Ball spindexerBall = Ball.NONE;

    private Intake.IntakeState intakeState = Intake.IntakeState.OFF;

    public void init(HardwareMap hardwareMap, Intake intake) {
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

        indexTimer = new ElapsedTime();
        indexTimer.reset();
        alignTimer = new ElapsedTime();
        alignTimer.reset();
        shootTimer = new ElapsedTime();
        shootTimer.reset();
    }

    /**
     * Main update loop - handles all spindexer logic and state transitions
     *
     * @param inButton - true when driver holds the "in" button
     * @param shootTrigger - value of shoot trigger (>0.1 = shooting)
     * @param isFlywheelReady - true when flywheel is at speed and ready to shoot
     */
    public void update(boolean inButton, double shootTrigger, boolean isFlywheelReady) {
        if (!powerOverride) {
            axonForward.setTargetRotation(target/GEAR_RATIO);
            axonForward.update();

            double power = axonForward.getPower();
            axonLeft.setPower(power);
            axonRight.setPower(power);
        }

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

        switch (spindexerState) {
            case INTAKING:
                intakeState = Intake.IntakeState.INTAKE;
                if (!hasEnteredIntaking) {
                    alignBack();
                    hasEnteredIntaking = true;
                }

                if (spindexerBall != Ball.NONE) {
                    if (spindexerBall == Ball.GREEN) {
                        order[0] = 2;
                    } else if (spindexerBall == Ball.PURPLE) {
                        order[0] = 1;
                    }
                    indexTimer.reset();
                    hasIndexed = false;
                    hasEnteredIntaking = false;
                    spindexerState = SpindexerState.INDEXING;
                } else if (!inButton) {
                    hasEnteredIntaking = false;
                    spindexerState = SpindexerState.READY_TO_SHOOT;
                }
                break;
            case INDEXING:
                intakeState = Intake.IntakeState.INTAKE;
                if (!hasIndexed) {
                    index();
                    hasIndexed = true;
                }
                if (indexTimer.seconds() > 0.5) {
                    if (isFull()) {
                        alignTimer.reset();
                        hasAligned = false;
                        spindexerState = SpindexerState.ALIGNING;
                    } else if (inButton) {
                        hasEnteredIntaking = true;
                        spindexerState = SpindexerState.INTAKING;
                    } else {
                        hasEnteredReadyToShoot = false;
                        spindexerState = SpindexerState.READY_TO_SHOOT;
                    }
                }
                break;
            case ALIGNING:
                intakeState = Intake.IntakeState.OUTTAKE;
                if (!hasAligned) {
                    align();
                    hasAligned = true;
                }
                if (alignTimer.seconds() > 1) {
                    hasEnteredReadyToShoot = false;
                    spindexerState = SpindexerState.READY_TO_SHOOT;
                }
                break;
            case READY_TO_SHOOT:
                intakeState = Intake.IntakeState.OUTTAKE;
                if (!hasEnteredReadyToShoot) {
                    alignToHold();
                    hasEnteredReadyToShoot = true;
                }
                if (inButton && !isFull()) {
                    hasEnteredReadyToShoot = false;
                    hasEnteredIntaking = false;
                    spindexerState = SpindexerState.INTAKING;
                } else if (shootTrigger > 0.2 && isFlywheelReady) {
                    shootTimer.reset();
                    hasShot = false;
                    hasEnteredReadyToShoot = false;
                    spindexerState = SpindexerState.SHOOTING;
                }
                break;
            case SHOOTING:
                intakeState = Intake.IntakeState.OUTTAKE;
                if (!hasShot) {
                    maxPower();
                    powerOverride = true;
                    hasShot = true;
                }
                if (shootTimer.seconds() > 1.2) {
                    powerOverride = false;
                   // setTargetAngle(getCurrentAngle());
                    order = new int[] {0, 0, 0};
                    hasShot = false;
                    hasEnteredReadyToShoot = false;
                    spindexerState = SpindexerState.READY_TO_SHOOT;
                }
                break;
        }
        intake.setState(intakeState);
    }

    public Intake.IntakeState getIntakeCommand() {
        return intakeState;
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
    public void alignBack() {
        target -= 60;
    }
    public void alignToHold() {
        target += 60;
    }

    public void resetTarget() {
        target = 0;
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

    public void setTargetAngle(double position) {
        target = position;
    }

    public double getCurrentAngle(){
        return (axonForward.getTotalRotation()) * GEAR_RATIO;
    }

    public double getTargetAngle(){
        return (axonForward.getTargetRotation()) * GEAR_RATIO;
    }

    public double getSensorAlphaSpin() {return sensorAlphaSpin;}

    public double getNormalizedRedSpin(){return trueRedSpin/sensorAlphaSpin;}

    public double getNormalizedBlueSpin(){return trueBlueSpin/sensorAlphaSpin;}

    public double getNormalizedGreenSpin(){return trueGreenSpin/sensorAlphaSpin;}

    public int getSpindexerPosForward(){
        double angle = Math.floorMod((int)getCurrentAngle(), 360);
        if ((angle <= 60 && angle >= 0) || ((angle - 360) >=-60 && (angle - 360) <= 0)){
            return 0;
        }
        if (angle <= 180 && angle >= 60){
            return 1;
        }
        if (angle <= 300 && angle >= 180){
            return 2;
        }
        return (int)angle;
    }
    public String getOrder(){
        return Arrays.toString(order);
    }

    public void maxPower(){
        axonForward.setPower(-1);
        axonRight.setPower(-1);
        axonLeft.setPower(-1);
    }

    public SpindexerState getState() {
        return spindexerState;
    }

    public void setState(SpindexerState spindexerState) {
        this.spindexerState = spindexerState;
    }

    public Ball getSpindexerBall() {
        return spindexerBall;
    }

    public void setCorrectOrder(int[] correctOrder) {
        this.correctOrder = correctOrder;
    }

    public int[] getOrderArray() {
        return order.clone();
    }

    // USELESS
    public boolean ballDetectedIntake(){
        return sensorAlphaIntake > 0.2;
    }

    public boolean ballIsGreenIntake() {
        return ballDetectedIntake() &&
                getNormalizedRedIntake() < 0.06 &&
                getNormalizedGreenIntake() > 0.1 &&
                getNormalizedBlueIntake() > 0.075;
    }
    public boolean ballIsPurpleIntake() {
        return ballDetectedIntake() &&
                getNormalizedRedIntake() > 0.06 &&
                getNormalizedGreenIntake() < 0.125 &&
                getNormalizedBlueIntake() > 0.1;
    }

    public double getTrueRedIntake(){return  trueRedIntake;}

    public double getTrueBlueIntake(){return  trueBlueIntake;}

    public double getTrueGreenIntake() {return trueGreenIntake;}

    public double getSensorAlphaIntake() {return sensorAlphaIntake;}

    public double getNormalizedRedIntake(){return trueRedIntake/sensorAlphaIntake;}

    public double getNormalizedBlueIntake(){return trueBlueIntake/sensorAlphaIntake;}

    public double getNormalizedGreenIntake(){return trueGreenIntake/sensorAlphaIntake;}

}
