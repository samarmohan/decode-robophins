package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.RTPAxon;

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
    
    public void init(HardwareMap hardwareMap){
        forwardEncoder = hardwareMap.get(AnalogInput.class, "encoderForward");
        crServoForward = hardwareMap.get(CRServo.class, "axonForward");
        axonForward = new RTPAxon(crServoForward, forwardEncoder);
        leftEncoder = hardwareMap.get(AnalogInput.class, "encoderLeft");
        crServoLeft = hardwareMap.get(CRServo.class, "axonLeft");
        axonLeft = new RTPAxon(crServoLeft, leftEncoder);
        rightEncoder = hardwareMap.get(AnalogInput.class, "encoderRight");
        crServoRight = hardwareMap.get(CRServo.class, "axonRight");
        axonRight = new RTPAxon(crServoForward, rightEncoder);
    }
    //updates RTPaxon PID and sets power to all three axons
    public void update(){
        axonForward.update();
        double power = axonForward.getPower();
        axonLeft.setPower(power);
        axonRight.setPower(power);
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
        axonForward.setTargetRotation(position/1.5);

    }

    public double getAngle(){
        return axonForward.getCurrentAngle()*1.5;
    }

    public double getRelativeAngle(){
        return axonForward.getCurrentAngle()-axonForward.getHomeAngle()*1.5;
    }
    public double getTarget(){
        return axonForward.getTargetRotation();
    }
}
