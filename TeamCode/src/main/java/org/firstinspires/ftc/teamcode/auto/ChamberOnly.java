package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.opencv.core.Point;

@Autonomous
public class ChamberOnly extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    SwerveDrive swerveDrive;
    DischargeSubsystem dischargeSubsystem;

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);

        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true, new Point(1.8, 0.2));
        register(swerveDrive, dischargeSubsystem);

        swerveDrive.bl.setHeading(0, false);
        swerveDrive.br.setHeading(0, false);
        swerveDrive.fl.setHeading(0, false);
        swerveDrive.fr.setHeading(0, false);
        schedule(new DischargeCommands.GoHomeCmd(dischargeSubsystem),
                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));
        while (opModeInInit()) {
            super.run();
            swerveDrive.bl.update();
            swerveDrive.br.update();
            swerveDrive.fl.update();
            swerveDrive.fr.update();
        }

        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SwerveCommands.GotoCmd(telemetry, swerveDrive, 1.8, 0.95, 0, 0.005, 0.2),
                        new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry)),
                new WaitCommand(2000),
                new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)));
    }

    @Override
    public void run() {
        telemetry.addData("pos", swerveDrive.getPosition());
        telemetry.addData("distance", swerveDrive.getDistance());
        telemetry.update();
        super.run();
    }
}
