package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outake {
    private PIDController controller;
    public static double p =0,i=0,d=0;
    public static double f = 0;

    public static int target = 0;
    private final double ticks_in_degree = 0;
    double armPos;
    double power;
    double pid;

    public DcMotorEx liftMotor;
    public Servo liftServo;
    ElapsedTime liftTimer = new ElapsedTime();

    public enum State {
        GROUND,
        EXTEND,
        DUMP,
        RETRACT
    }
    State currentState = State.GROUND;
    /*
     *  VALORILE PENTRU BRATUL DE RIDICARE
     */
    final int LIFT_DOWN = 0; // pozitia de jos
    final int LIFT_UP = 500; // pozitia de sus
    /*
    *   VALORILE PENTRU SERVO-UL CARE DEPOZITEAZA
     */
    final double DEPOSIT_IDLE = 0; // pozitia lui normala
    final double DEPOSIT_SCORING = 0.3; // pozitia lui cand arunca piesa

    // TIMPUL ALOCAT PENTRU CA SERVO-UL SA PUNA PIESA IN COS
    final double DUMP_TIME = 3;

    public void init(HardwareMap hardwareMap){
        controller = new PIDController(p,i,d);

        liftTimer.reset();

        liftMotor = hardwareMap.get(DcMotorEx.class,"LIFTMOTOR");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setTargetPosition(LIFT_DOWN);
        liftServo.setPosition(DEPOSIT_IDLE);
        SetState(State.GROUND);
    }

    public void SetState(State state) { currentState = state;}

    double calculatePID(){
        controller.setPID(p,i,d);
        armPos = liftMotor.getCurrentPosition();
        pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree))*f;
        power = pid + ff;
        return power;
    }

    public void Loop(Gamepad gp){
        liftMotor.setPower(calculatePID());

        switch (currentState){
            case GROUND:
                if(gp.x){
                    target = LIFT_UP;
                    currentState = State.EXTEND;
                }
                break;
            case EXTEND:
                if(Math.abs(liftMotor.getCurrentPosition()-LIFT_UP)<10) {

                    liftServo.setPosition(DEPOSIT_SCORING);
                    liftTimer.reset();
                    currentState = State.DUMP;
                }
                break;
            case DUMP:
                if(liftTimer.seconds() >= DUMP_TIME){
                    liftServo.setPosition(DEPOSIT_IDLE);
                    target = LIFT_DOWN;
                    currentState = State.RETRACT;
                }
                break;
            case RETRACT:
                if(Math.abs(liftMotor.getCurrentPosition()-LIFT_DOWN) < 10){
                    currentState = State.GROUND;
                }
                break;
            default:
                currentState = State.GROUND;
        }
        if(gp.y && currentState != State.GROUND){
            currentState = State.GROUND;
        }
    }


}
