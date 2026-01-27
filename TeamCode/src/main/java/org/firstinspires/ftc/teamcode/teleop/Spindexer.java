package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.axonTest.RTPAxon;

public class Spindexer {
    public AnalogInput forwardEncoder;

    public CRServo crServoForward;

    public RTPAxon axonForward;
    public AnalogInput rightEncoder;

    public CRServo crServoRight;

    public RTPAxon axonRight;
    public AnalogInput leftEncoder;

    public CRServo crServoLeft;

    public RTPAxon axonLeft;

    public NormalizedColorSensor intakeColor;

    public NormalizedColorSensor spinColor;

    private double trueRedIntake;
    private double trueBlueIntake;
    private double trueGreenIntake;
    private double sensorAlphaIntake;
    private double trueRedSpin;
    private double trueBlueSpin;

    private double trueGreenSpin;

    private double sensorAlphaSpin;

    public void init(HardwareMap hardwareMap){
        forwardEncoder = hardwareMap.get(AnalogInput.class, "encoderForward");
        crServoForward = hardwareMap.get(CRServo.class, "axonForward");
        axonForward = new RTPAxon(crServoForward, forwardEncoder);
        leftEncoder = hardwareMap.get(AnalogInput.class, "encoderLeft");
        crServoLeft = hardwareMap.get(CRServo.class, "axonLeft");
        axonLeft = new RTPAxon(crServoLeft, leftEncoder);
        rightEncoder = hardwareMap.get(AnalogInput.class, "encoderRight");
        crServoRight = hardwareMap.get(CRServo.class, "axonRight");
        axonRight = new RTPAxon(crServoRight, rightEncoder);
        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor.setGain(10);
        spinColor = hardwareMap.get(NormalizedColorSensor.class, "spinColor");
        spinColor.setGain(10);
    }
    //updates RTPaxon PID and sets power to all three axons
    //also updates color sensor values
    public void update(){
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

    public void setPIDCoefficients(double kp, double ki, double kd){
        axonForward.setPidCoeffs(kp, ki, kd);
    }

    public void setTargetPos(double position){
        axonForward.setTargetRotation(position);
    }

    public double getAngle(){
        return axonForward.getTotalRotation();
    }

    public double getRelativeAngle(){
        return axonForward.getCurrentAngle()-(axonForward.getHomeAngle());
    }
    public double getTarget(){
        return axonForward.getTargetRotation();
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

}
