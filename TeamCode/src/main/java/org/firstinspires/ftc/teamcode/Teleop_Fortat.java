package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop fortat", group="Linear OpMode")
public class Teleop_Fortat extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Gamepad gp1,gp2;
    private DcMotorEx liftMotor;


    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        liftMotor = hardwareMap.get(DcMotorEx.class,"LIFTMOTOR");

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gp1 = gamepad1;
        gp2 = gamepad1;

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            liftMotor.setPower(gp2.left_trigger - gp2.right_trigger);
            telemetry.addData("Putere Lift Motor",liftMotor.getPower());
            telemetry.addData("Pozitie Lift Motor",liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
