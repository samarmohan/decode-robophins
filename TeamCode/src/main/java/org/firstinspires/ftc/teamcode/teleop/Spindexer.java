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

    public AnalogInput forwardEncoder;
    public AnalogInput leftEncoder;
    public AnalogInput rightEncoder;

    public CRServo crServoForward;
    public CRServo crServoLeft;
    public CRServo crServoRight;


    public Axon axonForward;
    public Axon axonLeft;
    public Axon axonRight;

    public NormalizedColorSensor intakeColor;
    public NormalizedColorSensor spinColor;

    private double sensorAlphaIntake;
    private double sensorAlphaSpin;

    private double trueRedIntake;
    private double trueBlueIntake;
    private double trueGreenIntake;
    private double trueRedSpin;
    private double trueBlueSpin;
    private double trueGreenSpin;
    private boolean ballInIntake;
    private boolean ballInSpin;
    public double target;
    private boolean powerOverride = false;
    private boolean hasShot;
    private boolean hasIndexed;
    private int[] order = {0,0,0};
    private int[] correctOrder = {2,1,1};
    private double[][] weights = generateWeightArray(0.9);

    private ElapsedTime indexTimer;
    private ElapsedTime alignTimer;
    private ElapsedTime shootTimer;


    private enum State{
        INTAKING,
        INDEXING,
        ALIGNING,
        READY_TO_SHOOT,
        SHOOTING,
        HOLDING
    }

    private enum Ball {
        NONE,
        PURPLE,
        GREEN
    }

    public State state = State.ALIGNING;
    public Ball spindexerBall = Ball.NONE;
    public Ball intakeBall = Ball.NONE;

    public void init(HardwareMap hardwareMap){
        forwardEncoder = hardwareMap.get(AnalogInput.class, "encoderForward");
        leftEncoder = hardwareMap.get(AnalogInput.class, "encoderLeft");
        rightEncoder = hardwareMap.get(AnalogInput.class, "encoderRight");

        crServoForward = hardwareMap.get(CRServo.class, "axonForward");
        crServoLeft = hardwareMap.get(CRServo.class, "axonLeft");
        crServoRight = hardwareMap.get(CRServo.class, "axonRight");

        axonForward = new Axon(crServoForward, forwardEncoder);
        axonLeft = new Axon(crServoLeft, leftEncoder);
        axonRight = new Axon(crServoRight, rightEncoder);

        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        spinColor = hardwareMap.get(NormalizedColorSensor.class, "spinColor");

        intakeColor.setGain(10);
        spinColor.setGain(10);

        indexTimer = new ElapsedTime();
        indexTimer.reset();
        alignTimer = new ElapsedTime();
        alignTimer.reset();
        shootTimer = new ElapsedTime();
        shootTimer.reset();
    }
    //updates RTPaxon PID and sets power to all three axons
    //also updates color sensor values
    public void update(boolean in, boolean out, boolean off, double shoot){
        if(!powerOverride) {
            axonForward.setTargetRotation(target/GEAR_RATIO);
            axonForward.update();

            double power = axonForward.getPower();

            axonLeft.setPower(power);
            axonRight.setPower(power);
        }

        NormalizedRGBA colorsIntake = intakeColor.getNormalizedColors();
        NormalizedRGBA colorsSpin = spinColor.getNormalizedColors();

        trueRedIntake = colorsIntake.red;
        trueBlueIntake = colorsIntake.blue;
        trueGreenIntake = colorsIntake.green;
        sensorAlphaIntake = colorsIntake.alpha;

        trueRedSpin = colorsSpin.red;
        trueBlueSpin = colorsSpin.blue;
        trueGreenSpin = colorsSpin.green;
        sensorAlphaSpin = colorsSpin.alpha;

        //spindexer logic
        if (ballDetectedIntake()) {
            ballInIntake = true;
        }
        else{
            ballInIntake = false;
            intakeBall = Ball.NONE;
        }
        if (ballInIntake && ballIsGreenIntake()) {
            intakeBall = Ball.GREEN;
        }
        else if (ballIsPurpleIntake()) {
            intakeBall = Ball.PURPLE;
        }
        if (ballDetectedSpin()) {
            ballInSpin = true;
        }
        else{
            ballInSpin = false;
            spindexerBall = Ball.NONE;
        }
        if (ballInSpin && ballIsGreenSpin()) {
            spindexerBall = Ball.GREEN;
        }
        else if (ballIsPurpleSpin()) {
            spindexerBall = Ball.PURPLE;
        }

        switch (state) {
            case INTAKING:
                if (!in) {
                    state=State.ALIGNING;
                    alignTimer.reset();
                }
                if (spindexerBall != Ball.NONE && intakeBall != spindexerBall) {                    //ball is now in spindexer
                    if (spindexerBall == Ball.GREEN) {
                        hasIndexed = false;
                        order[0] = 2;
                        state = State.INDEXING;
                        indexTimer.reset();
                    }
                    if (spindexerBall == Ball.PURPLE) {
                        hasIndexed = false;
                        order[0] = 1;
                        state=State.INDEXING;
                        indexTimer.reset();
                    }
                }
                break;
            case INDEXING:
                if(indexTimer.seconds() > 0.3){
                    if(!hasIndexed){
                        index();
                        hasIndexed = true;
                    }
                    if(indexTimer.seconds() > 0.8) {
                        if (!isFull()) {
                            state = State.INTAKING;
                        }
                        else{
                            align();
                            state=State.ALIGNING;
                            alignTimer.reset();
                        }
                    }
                }
                break;
            case ALIGNING:
                if(alignTimer.seconds() > 1){
                    state = State.READY_TO_SHOOT;
                    alignToHold();
                }

                break;
            case READY_TO_SHOOT:
                if (!isFull() && in){
                    state = State.INTAKING;
                    alignBack();
                }
                else if (shoot > 0.1){
                    state=State.SHOOTING;
                    shootTimer.reset();
                    hasShot = false;
                    alignBack();
                }
                break;
            case SHOOTING:
                if(shootTimer.seconds() > 0.1){
                    if(!hasShot) {
                        maxPower();
                        powerOverride = true;
                        hasShot = true;
                    }
                    if(shootTimer.seconds() > 0.7) {
                        powerOverride = false;
                        setTargetAngle(getCurrentAngle());
                        order = new int[] {0, 0, 0};
                        state = State.INTAKING;
                        hasShot = false;
                    }
                }
                break;
        }
    }

    public void index(){
        target += 120;
        shiftArrayRight(order);
    }
    public void align(){
        int shift = findBestShift(correctOrder, order, weights);
        for (int i = 0; i < shift; i++){
            target += 120;
            shiftArrayRight(order);
        }
    }
    public void shoot(){
        target -= 480;
    }
    public void alignBack(){
        target -= 60;
    }
    public void alignToHold(){
        target += 60;
    }

    public void resetTarget(){
        target = 0;
    }


    public boolean isFull(){
        for (int i : order){
            if (i == 0){
                return false;
            }
        }
        return true;
    }
    public boolean ballDetectedIntake(){
        return sensorAlphaIntake > 0.2;
    }


    public boolean ballIsGreenIntake(){
        return ballDetectedIntake() &&
                getNormalizedRedIntake() < 0.06 &&
                getNormalizedGreenIntake() > 0.1 &&
                getNormalizedBlueIntake() > 0.075;
    }

    public boolean ballIsPurpleIntake(){
        return ballDetectedIntake() &&
                getNormalizedRedIntake() > 0.06 &&
                getNormalizedGreenIntake() < 0.125 &&
                getNormalizedBlueIntake() > 0.1;
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
    public static void shiftArrayLeft (int array[]) {
        int lastIndex = array.length - 1;
        int oldFirst = array[0];
        for (int i = 0; i < lastIndex; i++) {
            array[i] = array[i + 1];
        }
        array[lastIndex] = oldFirst;
    }
    //shifts array 1 index to the right ([a,b,c]->[c,a,b])
    public static void shiftArrayRight (int array[]) {
        int lastIndex = array.length - 1;
        int oldLast = array[lastIndex];
        for (int i = lastIndex; i > 0; i--) {
            array[i] = array[i - 1];
        }
        array[0] = oldLast;
    }

    public void setTargetAngle(double position){
        target = position;
    }

    public double getCurrentAngle(){
        return (axonForward.getTotalRotation()) * GEAR_RATIO;
    }

    public double getTargetAngle(){
        return (axonForward.getTargetRotation()) * GEAR_RATIO;
    }
    public double getTrueRedIntake(){return  trueRedIntake;}

    public double getTrueBlueIntake(){return  trueBlueIntake;}

    public double getTrueGreenIntake() {return trueGreenIntake;}

    public double getSensorAlphaIntake() {return sensorAlphaIntake;}

    public double getNormalizedRedIntake(){return trueRedIntake/sensorAlphaIntake;}

    public double getNormalizedBlueIntake(){return trueBlueIntake/sensorAlphaIntake;}

    public double getNormalizedGreenIntake(){return trueGreenIntake/sensorAlphaIntake;}

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
}
