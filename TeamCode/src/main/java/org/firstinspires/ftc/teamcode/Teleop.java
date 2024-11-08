package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOP", group="Linear OpMode")
@Disabled
public class Teleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Gamepad gp1,gp2;
    private Outake outake;
    private Intake intake;



    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        outake = new Outake();
        intake = new Intake();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outake.init(hardwareMap);
        intake.init(hardwareMap);
        gp1 = gamepad1;
        gp2 = gamepad1;

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            intake.Loop(gp2);
            outake.Loop(gp2);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
