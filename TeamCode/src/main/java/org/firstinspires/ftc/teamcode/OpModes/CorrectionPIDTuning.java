package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.Pose;

@Disabled
@Config
@Autonomous(name = "Correction PID Tuning")
public class CorrectionPIDTuning extends LinearOpMode {

    Hardware hardware;

    MecanumDrive drive;
    Localizer localizer;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);

        localizer = new Localizer(hardware, new Pose());
        drive = new MecanumDrive(hardware, localizer, MecanumDrive.RunMode.PID);

        waitForStart();

        localizer.imu.startIMUThread(this);

        drive.setTargetPose(new Pose());

        while(opModeIsActive() && !isStopRequested()){
            localizer.update();
            drive.update();

            telemetry.addData("Pose X", localizer.getPoseEstimate().getX());
            telemetry.addData("Pose Y", localizer.getPoseEstimate().getY());
            telemetry.addData("Pose Heading", localizer.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
