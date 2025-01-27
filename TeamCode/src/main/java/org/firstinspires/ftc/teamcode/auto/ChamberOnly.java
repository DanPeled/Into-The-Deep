package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
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

@Autonomous(group = "chamber")
public class ChamberOnly extends CommandOpMode {
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
        //schedule(new DischargeCommands.GoHomeCmd(dischargeSubsystem),
        //        new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));
        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);
        while (opModeInInit()) {
            super.run();
        }

//        schedule(new SequentialCommandGroup(
//                new ParallelCommandGroup(
//                        new SwerveCommands.GotoCmd(telemetry, swerveDrive, 1.8, 0.95, 0, 0.005, 7, 0.2),
//                        new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry)),
//                new WaitCommand(2000),
//                new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)));
//


        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 1.02, 0, 0.05, 0.9, true)),
//                        new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry)),
                new WaitCommand(200),
                new ParallelCommandGroup(
//                        new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry),
                ),
                new WaitCommand(200),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 0.8, 0, 0.14, 1),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.76, 0.8, 0, 0.1, 0.8),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.7, 1.5, 0, 0.04, 0.8),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.93, 1.5, 0, 0.05, 0.9),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.93, 0.4, 0, 0.05, 0.9, true),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.98, 0.65, 0, 0.008, 1),
                new WaitCommand(1500),
                new IntakeCommands.StartIntakeCmd(intakeSubsystem),
                new WaitCommand(300),
                new IntakeCommands.SampleIntakeCmd(intakeSubsystem),
                new WaitCommand(300),

                new ParallelCommandGroup(
                        new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 0.6, 0, 0.06, 1)
                ),

//                new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry),
                new WaitCommand(1000),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.78, 1.02, 0, 0.05, 0.9),
                new ParallelCommandGroup(
                        new IntakeCommands.StartIntakeCmd(intakeSubsystem)
//                new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry),
                ),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3.005, 0.655, 0, 0.008, 0.7),
                new WaitCommand(1000),
                new IntakeCommands.SampleIntakeCmd(intakeSubsystem),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 0.6, 0, 0.06, 1)
                ),

//                new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry),
                new WaitCommand(1000),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.78, 1.02, 0, 0.05, 0.9)
                // DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)
        ));
//        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.8, 1.5, 0, 0.05, 1, true),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3.4, 1.5, 0, 0.02, 0.8, true),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3.4, 0.25, 0, 0.02, 0.7, true)
//        new ParallelRaceGroup(new InstantCommand(() -> mecanumDrive.drive(0, 0.4, 0, 0.2)),
//                new WaitCommand(350)),
    }

    @Override
    public void run() {

        telemetry.update();
        super.run();
    }
}
