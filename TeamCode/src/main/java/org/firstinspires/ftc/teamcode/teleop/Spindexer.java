package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private double target = 0;
    private boolean indexing = false;

    private int[] order = {0,0,0};

    private int[] correctOrder = {2,1,1};
    private double[][] weights = generateWeightArray(0.9);

    private ElapsedTime indexTimer;
    private ElapsedTime alignTimer;


    private enum State{
        INTAKING,
        INDEXING,
        ALIGNING,
        READY_TO_SHOOT,
        SHOOTING,
        HOLDING
    }

    private enum ball{
        NONE,
        PURPLE,
        GREEN
    }

    private State state= State.ALIGNING;
    private ball currentBallIntake = ball.NONE;
    private ball lastBallIntake = ball.NONE;

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
    }
    //updates RTPaxon PID and sets power to all three axons
    //also updates color sensor values
    public void update(boolean in, boolean out, boolean off, double shoot){
        axonForward.setTargetRotation(target);
        axonForward.update();

        double power = axonForward.getPower();

        axonLeft.setPower(power);
        axonRight.setPower(power);

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
        if (ballDetectedIntake()){
            ballInIntake = true;
        }
        else{
            ballInIntake = false;
            currentBallIntake = ball.NONE;
        }
        if (ballInIntake && ballIsGreenIntake()){
            currentBallIntake = ball.GREEN;
        }
        else if (ballIsPurpleIntake()){
            currentBallIntake = ball.PURPLE;
        }

        switch (state){
            case INTAKING:
                if(out){
                    state=State.ALIGNING;
                }
                if ((lastBallIntake == ball.GREEN || lastBallIntake == ball.PURPLE) && currentBallIntake == ball.NONE) {
                    //ball is now in spindexer
                    if (lastBallIntake == ball.GREEN) {
                        order[getSpindexerPosForward()] = 2;
                        state = State.INDEXING;
                        indexTimer.reset();
                    }
                    if (lastBallIntake == ball.PURPLE) {
                        order[getSpindexerPosForward()] = 1;
                        state=State.INDEXING;
                        indexTimer.reset();
                    }
                }
                break;
            case INDEXING:
                if(indexTimer.seconds() > 0.3){
                    index();
                    if(indexTimer.seconds() > 0.5) {
                        if (order[getSpindexerPosForward()] == 0) {
                            state = State.INTAKING;
                        }
                        else{
                            state=State.ALIGNING;
                        }
                    }
                }
                break;
            case ALIGNING:
                if(alignTimer.seconds() > 0.8){
                    state = State.READY_TO_SHOOT;
                }
                break;
            case READY_TO_SHOOT:
                if (shoot > 0.1){
                    state=State.SHOOTING;
                }
                break;
            case SHOOTING:
                shoot();
                state = State.ALIGNING;
                break;
        }

        lastBallIntake = currentBallIntake;
    }

    public void index(){
        target -= 120;
        shiftArrayLeft(order);
    }
    public void align(){
        int shift = findBestShift(correctOrder, order, weights);
        for (int i = 0; i >shift; i++){
            target -= 120;
        }
    }
    public void shoot(){
        target += 480;
        order = new int[] {0, 0, 0};
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
                getNormalizedRedSpin() < 0.125 &&
                getNormalizedGreenSpin() > 0.2 &&
                getNormalizedBlueSpin() > 0.1;
    }

    public boolean ballIsPurpleSpin(){
        //not tuned
        return ballDetectedSpin() &&
                getNormalizedRedSpin() > 0.1 &&
                getNormalizedGreenSpin() < 0.25 &&
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
            shiftArrayLeft(balls);
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
        target = position/GEAR_RATIO;
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
        double angle = getCurrentAngle() % 360;
        if (angle < 10 && angle >-10){
            return 0;
        }
        if (angle < 130 && angle > 110){
            return 1;
        }
        if (angle < -110 && angle > -130){
            return 2;
        }
        return -1;
    }
}
