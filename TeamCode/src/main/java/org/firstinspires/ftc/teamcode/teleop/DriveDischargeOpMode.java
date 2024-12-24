package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

public class DriveDischargeOpMode extends CommandOpMode {
    GamepadEx systemGamepad;
    GamepadEx driverGamepad;
    private DischargeSubsystem dischargeSubsystem;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    DischargeCommands.DischargePowerCmd dischargePowerCmd;
    private ElapsedTime runtime = new ElapsedTime();
    private SwerveDrive swerveDrive;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this);
        register(swerveDrive);

//        CommandScheduler.getInstance().setDefaultCommand(swerveDrive,new SwerveCommands.PowerCmd(telemetry, swerveDrive, driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX, () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        systemGamepad = new GamepadEx(gamepad2);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        register(dischargeSubsystem);
        dischargeSubsystem.setDefaultCommand(new DischargeCommands.DischargePowerCmd(() -> systemGamepad.getLeftY(), dischargeSubsystem, telemetry));
        Button dPadUp = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_UP);
        Button dPadDown = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_DOWN);
        Button dPadRight = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_RIGHT);
        Button dPadLeft = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_LEFT);
        Button leftBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.LEFT_BUMPER);
        Button rightBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        Button A = new GamepadButton(systemGamepad, GamepadKeys.Button.A);

        dPadUp.whenPressed(new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, 1600, 10, telemetry));
        dPadDown.whenPressed(new DischargeCommands.GoHomeCmd(dischargeSubsystem));
        dPadRight.whenPressed(new DischargeCommands.GearBoxSwapCmd(dischargeSubsystem));
        dPadLeft.whenPressed(new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));
        leftBumper.whenPressed(new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem));
        rightBumper.whenPressed(new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));
        A.whileHeld(new DischargeCommands.DischargeClawTestCmd(() -> systemGamepad.getRightX(), dischargeSubsystem, telemetry));

        schedule(new DischargeCommands.GoHomeCmd(dischargeSubsystem));

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
        }
    }

}