package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargePowerCmd;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargeGotoCmd;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargeReleaseCmd;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargeGrabCmd;

@TeleOp
public class BasicDischargeTest extends CommandOpMode {
    GamepadEx systemGamepad;
    private DischargeSubsystem dischargeSubsystem;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    DischargePowerCmd dischargePowerCmd;

    @Override
    public void initialize() {
        systemGamepad = new GamepadEx(gamepad2);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        register(dischargeSubsystem);
        dischargeSubsystem.setDefaultCommand(new DischargePowerCmd(() -> systemGamepad.getLeftY(), dischargeSubsystem, telemetry));
        Button dPadUp = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_UP);
        Button dPadDown = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_DOWN);
        Button dPadRight = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_RIGHT);
        Button dPadLeft = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_LEFT);
        Button leftBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.LEFT_BUMPER);
        Button rightBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        Button A = new GamepadButton(systemGamepad, GamepadKeys.Button.A);

        dPadUp.whenPressed(new DischargeGotoCmd(dischargeSubsystem, 1600, 10, telemetry));
        dPadDown.whenPressed(new DischargeCommands.GoHomeCmd(dischargeSubsystem));
        dPadLeft.whenPressed(new DischargeGrabCmd(dischargeSubsystem));
        leftBumper.whenPressed(new DischargeReleaseCmd(dischargeSubsystem));
        rightBumper.whenPressed(new DischargeGrabCmd(dischargeSubsystem));
        A.whileHeld(new DischargeCommands.DischargeClawTestCmd(() -> systemGamepad.getRightX(), dischargeSubsystem, telemetry));

        schedule(new DischargeCommands.GoHomeCmd(dischargeSubsystem));

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
        }
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("yPower", systemGamepad.getLeftY() * 0.75);
        telemetry.addData("posInCM", dischargeSubsystem.getLiftPosInCM());
        telemetry.addData("pos", dischargeSubsystem.getPosition());
        telemetry.addData("pos2", dischargeSubsystem.getPosition2());
        telemetry.addData("gearRatio", dischargeSubsystem.getGearBoxRatio());
        telemetry.addData("timeUp", dischargeSubsystem.timeUp);

        telemetry.update();
    }
}