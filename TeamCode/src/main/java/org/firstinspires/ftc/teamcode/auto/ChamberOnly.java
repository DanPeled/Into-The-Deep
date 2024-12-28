package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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

        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true,new Point(2.33,0.2));
        schedule(new SequentialCommandGroup(
                new SwerveCommands.GotoCmd(telemetry,swerveDrive,1.76,1.06,0.0,0.05,0.2),
                new SwerveCommands.SetPosition(swerveDrive,new Point(swerveDrive.getPosition().x, 1.06)),
                new SwerveCommands.GotoCmd(telemetry,swerveDrive,1.76,1.01,0.0,0.01,0.2),
                new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry),//discharge get to chamber pos
                new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry),//chamber discharge
                new SwerveCommands.GotoCmd(telemetry,swerveDrive,2.33,0.2,0.0,0.05,0.2),
                new SwerveCommands.GotoCmd(telemetry,swerveDrive,3.33,0.2,0.0,0.05,0.2)));
    }


}
