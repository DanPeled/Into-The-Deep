package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.opencv.core.Point;

@Disabled
@Autonomous
public class BasketOnly extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    SwerveDrive swerveDrive;
    DischargeSubsystem dischargeSubsystem;

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);

        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true, new Point(0.82, 0.2));
        register(swerveDrive, dischargeSubsystem);
        swerveDrive.setInitialHeading(180);
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

        schedule(new SwerveCommands.GotoCmd(telemetry, swerveDrive, 0.82, 0.8, 0, 0.03, -1, 0.2),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 0.0, 0.8, 0, 0.03, -1, 0.2),
                new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highBasketHeight, telemetry),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 0.0, 0.55, 0, 0.03, -1, 0.2),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                new SwerveCommands.GotoCmd(telemetry, swerveDrive, 0.0, 0.8, 0, 0.03, -1, 0.2),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
        );
    }
}
