package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ThreeDeadWheelLocalizer;

@TeleOp
public class MecanumOpMode extends CommandOpMode {
    MecanumDrive mecanumDrive;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    GamepadEx driverGamepad;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, this);
        register(mecanumDrive);
        mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
                () -> driverGamepad.getLeftX(), () -> driverGamepad.getLeftY(), () -> driverGamepad.getRightX(),
                () -> 0.5 + 0.5 * driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.addData("encoder bl", mecanumDrive.bl.getCurrentPosition());
        telemetry.addData("encoder br", mecanumDrive.br.getCurrentPosition());
        telemetry.addData("encoder fl", mecanumDrive.fl.getCurrentPosition());
        telemetry.addData("encoder fr", mecanumDrive.fr.getCurrentPosition());
        telemetry.addData(";llllll", mecanumDrive.getPosition());
        telemetry.update();
    }
}
