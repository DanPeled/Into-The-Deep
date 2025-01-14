package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
    GamepadEx gamepad;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    MultipleTelemetry multipleTelemetry;

    @Override
    public void initialize() {
        GamepadEx gamepad = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true, new Point(1.8, 0.2));

        register(swerveDrive);

//        schedule(new SequentialCommandGroup(
//                new GotoCmd(telemetry, swerveDrive, -0.66,0.0,0.0, 0.01,0.2),
//                new GotoCmd(telemetry, swerveDrive, 0,0.0,0.0, 0.01,0.2)));
//        schedule(new GotoCmd(telemetry, swerveDrive, 0.67, 0.21, 0, 0.03, 0.2));
        Button a = new GamepadButton(gamepad, GamepadKeys.Button.A);
        a.whileHeld(new SwerveCommands.SetRotationCmd(swerveDrive, 0));
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
