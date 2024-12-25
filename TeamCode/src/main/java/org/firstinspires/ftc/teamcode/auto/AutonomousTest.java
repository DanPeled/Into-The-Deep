package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands.GotoCmd;
@Autonomous
public class AutonomousTest extends CommandOpMode {
    private SwerveDrive swerveDrive;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    @Override
    public void initialize() {
        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry,this, true);
        register(swerveDrive);
        schedule(new GotoCmd(telemetry, swerveDrive, 0.5,0.5,27, 0.1,0.2));
    }

    @Override
    public void run() {
        super.run();
    }
}
