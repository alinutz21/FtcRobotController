package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Intake Tunning",group = "Tests")
public class Intake_WIRELESS extends LinearOpMode {
    private PIDController controller;
    public static double p =0,i=0,d=0;
    public static double f = 0;

    public static int target = 0;
    private final double ticks_in_degree = 0;
    double armPos;
    double power;
    double pid;

    public DcMotorEx liftMotor;

    @Override
    public void runOpMode(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftMotor = hardwareMap.get(DcMotorEx.class,"LIFTMOTOR");


        waitForStart();
        while(opModeIsActive()){
            controller.setPID(p,i,d);
            int liftPos = liftMotor.getCurrentPosition();
            double pid = controller.calculate(armPos,target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

            double power = pid + ff;
            liftMotor.setPower(power);

        }
    }




    double calculatePID(){
        controller.setPID(p,i,d);
        armPos = liftMotor.getCurrentPosition();
        pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree))*f;
        power = pid + ff;
        return power;
    }





}
