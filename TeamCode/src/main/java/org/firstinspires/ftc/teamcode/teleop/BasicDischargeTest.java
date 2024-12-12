package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargePowerCmd;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargeGotoCmd;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargeReleaseCmd;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargeHoldCmd;
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
        Button dPadUp = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_UP);
        dPadUp.whenPressed(new DischargeGotoCmd(dischargeSubsystem,1000,10,telemetry));
        Button dPadDown = new GamepadButton(systemGamepad,GamepadKeys.Button.DPAD_DOWN);
        dPadDown.whenPressed(new DischargeGotoCmd(dischargeSubsystem,300,10,telemetry));
        Button dPadRight = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_RIGHT);
        dPadRight.whenPressed(new DischargeReleaseCmd(dischargeSubsystem));
        Button dPadLeft = new GamepadButton(systemGamepad,GamepadKeys.Button.DPAD_DOWN);
        dPadLeft.whenPressed(new DischargeHoldCmd(dischargeSubsystem));

    }

    @Override
    public void run() {
//        dischargeSubsystem.setPosition(400);
        super.run();

        telemetry.addData("pos",dischargeSubsystem.getPosition());
        telemetry.update();
    }
}