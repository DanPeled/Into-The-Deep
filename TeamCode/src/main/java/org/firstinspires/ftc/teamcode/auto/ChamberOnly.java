package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.opencv.core.Point;

@Autonomous
public class ChamberOnly extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    SwerveDrive swerveDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);

        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true, new Point(1.8, 0.2));
        register(swerveDrive, dischargeSubsystem);

        swerveDrive.bl.setHeading(0, false);
        swerveDrive.br.setHeading(0, false);
        swerveDrive.fl.setHeading(0, false);
        swerveDrive.fr.setHeading(0, false);
        //schedule(new DischargeCommands.GoHomeCmd(dischargeSubsystem),
        //        new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));
        schedule(new SequentialCommandGroup(
                new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem),
                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
                new IntakeCommands.ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
                //new IntakeCommands.Wait(intakeSubsystem, 1),
                new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true),
                new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.TRANSFER),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)));
        while (opModeInInit()) {
            super.run();
            swerveDrive.bl.update();
            swerveDrive.br.update();
            swerveDrive.fl.update();
            swerveDrive.fr.update();
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
                        new SwerveCommands.GotoCmd(telemetry, swerveDrive, 1.8, 0.97, 0, 0.005, 6.5, 0.2),
                        new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry)),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry),
                        new InstantCommand(() -> swerveDrive.drive(0, 0.1, 0, 0.2))),
                new WaitCommand(500),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 1.8, 0.8, 0, 0.04, 0.2),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 2.75, 0.8, 0, 0.03, 0.2),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 2.75, 1.5, 0, 0.02, 0.2),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 2.9, 1.5, 0, 0.01, 0.2),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 2.9, 0.25, 0, 0.02, 0.2, true),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 2.8, 1.5, 0, 0.02, 0.2, true),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 3.1, 1.5, 0, 0.02, 0.2, true),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 3.1, 0.25, 0, 0.02, 0.2, true)));
    }

    @Override
    public void run() {
        telemetry.addData("pos", swerveDrive.getPosition());
        telemetry.addData("distance", swerveDrive.getDistance());
        telemetry.update();
        super.run();
    }
}
