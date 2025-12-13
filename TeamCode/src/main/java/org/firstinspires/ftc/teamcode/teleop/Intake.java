package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor frontIntake;
    private DcMotor backIntake;

    public enum State{
        IN,
        OUT,
        SHOOT,
        OFF
    }
    private State state = State.OFF;


    public void init(HardwareMap hardwareMap){
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");

        //positive is in, negative is out
        frontIntake.setDirection(DcMotor.Direction.REVERSE);
        //positive is up, negative is down
        backIntake.setDirection(DcMotor.Direction.FORWARD);
    }

    public void in(){
        frontIntake.setPower(1.0);
        backIntake.setPower(-1.0);
    }
    public void out(){
        frontIntake.setPower(-1.0);
        backIntake.setPower(-1.0);
    }
    public void off(){
        frontIntake.setPower(0.0);
        backIntake.setPower(0.0);
    }
    public void shoot(){
        frontIntake.setPower(1.0);
        backIntake.setPower(1.0);
    }
    public void loop(boolean inOrOut, boolean onOrOff, double shouldShoot){
        switch (state){
            case IN:
                in();
                if (inOrOut){
                    setState(State.OUT);
                }
                if (onOrOff){
                    setState(State.OFF);
                }
                if (shouldShoot > 0.2){
                    setState(State.SHOOT);
                }
                break;
            case OUT:
                out();
                if (inOrOut){
                    setState(State.IN);
                }
                if (onOrOff){
                    setState(State.OFF);
                }
                if (shouldShoot > 0.2){
                    setState(State.SHOOT);
                }
                break;
            case OFF:
                off();
                if (onOrOff){
                    setState(State.IN);
                }
                if (shouldShoot > 0.2){
                    setState(State.SHOOT);
                }
                break;
            case SHOOT:
                shoot();
                if (shouldShoot < 0.1){
                    setState(State.IN);
                }
                break;
            default:
                off();
                break;
        }
    }
    public void setState(State state){
        this.state = state;
    }
    public State getState(){
        return state;
    }
}
