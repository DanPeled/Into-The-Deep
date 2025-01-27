package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.opencv.core.Point;

public class ChamberOnePark extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(1.8, 0.2), 0, this);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem);
        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);


        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
        }


        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        AutoUtils.chamberGoto(mecanumDrive, telemetry),
                        AutoUtils.dischargeGotoChamber(dischargeSubsystem, telemetry)
                ),
                AutoUtils.chamberDischarge(dischargeSubsystem,telemetry),
                new WaitCommand(20000),
                AutoUtils.inwardsPark(mecanumDrive, telemetry)//maybe should be outwards
        ));
    }
}
