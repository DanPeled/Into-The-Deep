package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class MecanumOpMode extends CommandOpMode {
    MecanumDrive mecanumDrive;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

    @Override
    public void initialize() {
        mecanumDrive = new MecanumDrive(multipleTelemetry,hardwareMap);
        register(mecanumDrive);
        mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry,mecanumDrive,
                () -> (double)gamepad1.left_stick_x,() -> -(double)gamepad1.left_stick_y,() -> (double)gamepad1.right_stick_x,
                () -> 0.1 + 0.9 * gamepad1.right_trigger,true));
    }
}
