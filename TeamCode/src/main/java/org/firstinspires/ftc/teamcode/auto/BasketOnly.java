package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.opencv.core.Point;

@Autonomous
public class BasketOnly extends CommandOpMode {
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
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(0.82, 0.2), 180, this);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem);
        mecanumDrive.setHeading(0);
        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);

        while (opModeInInit()) {
            super.run();

        }

        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highBasketHeight, telemetry),
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.2, 0.8, 180, 0.05, 0.5)),
                new ParallelCommandGroup(
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.2, 0.35, 180, 0.03, 0.5),
                        new IntakeCommands.StartIntakeCmd(intakeSubsystem)),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.3, 0.8, 180, 0.005, 0.65),
                new ParallelRaceGroup(
                        new DischargeCommands.GoHomeCmd(dischargeSubsystem),
                        new IntakeCommands.SampleIntakeCmd(intakeSubsystem)
                ),
                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highBasketHeight, telemetry),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.2, 0.35, 180, 0.03, 0.5),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.3, 0.8, 180, 0.005, 0.65),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)

        ));
    }
}
