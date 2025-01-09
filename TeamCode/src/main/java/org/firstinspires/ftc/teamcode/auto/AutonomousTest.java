package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands.GotoCmd;
import org.opencv.core.Point;

@Autonomous
public class AutonomousTest extends CommandOpMode {
    private SwerveDrive swerveDrive;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

    @Override
    public void initialize() {
        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true, new Point(1.8, 0.2));

        register(swerveDrive);
//        schedule(new SequentialCommandGroup(
//                new GotoCmd(telemetry, swerveDrive, -0.66,0.0,0.0, 0.01,0.2),
//                new GotoCmd(telemetry, swerveDrive, 0,0.0,0.0, 0.01,0.2)));
        schedule(new GotoCmd(telemetry, swerveDrive, 0.67, 0.21, 0, 0.03, 0.2));
    }

    @Override
    public void run() {
        multipleTelemetry.addData("x", swerveDrive.getPosition().x);
        multipleTelemetry.addData("y", swerveDrive.getPosition().y);
        multipleTelemetry.addData("bl pos", swerveDrive.bl.getPosition());
        multipleTelemetry.addData("br pos", swerveDrive.br.getPosition());
        multipleTelemetry.addData("fl pos", swerveDrive.fl.getPosition());
        multipleTelemetry.addData("fr pos", swerveDrive.fr.getPosition());
        super.run();
    }
}
