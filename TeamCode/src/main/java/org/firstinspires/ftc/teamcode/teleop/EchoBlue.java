package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.commands.SetStateCommands;
import org.firstinspires.ftc.teamcode.subsystems.Pipelines;

@TeleOp
public class EchoBlue extends Echo {
    @Override
    public void noneBindings() {
        systemLeftStick.whenPressed(new DischargeCommands.ResetDischarge(dischargeSubsystem));
        systemRightBumper.whenPressed(new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem));

        driverDPadDown.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> -0.2, () -> 0.0,
                () -> 0.3, true));
        driverDPadUp.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> 0.2, () -> 0.0,
                () -> 0.3, true));
        driverDPadLeft.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> -0.2, () -> 0.0, () -> 0.0,
                () -> 0.3, true));
        driverDPadRight.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.2, () -> 0.0, () -> 0.0,
                () -> 0.3, true));


        mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
                mecanumX, mecanumY, mecanumR, ()
                -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5 + 0.5, true));

        intakeSubsystem.setDefaultCommand(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem, systemGamepad::getLeftY));


        driverA.whenHeld(new MecanumCommands.SetRotationCmd(mecanumDrive, 0));

        systemA.whenPressed(new SequentialCommandGroup(
                new IntakeCommands.WaitForTransferEnd(),
                new SetStateCommands.ChamberStateCmd(), //change to chamber state
                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)));


        systemY.whenPressed(new SequentialCommandGroup(
                new IntakeCommands.WaitForTransferEnd(),
                new SetStateCommands.BasketStateCmd(), //change to basket state
                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highBasketHeight)));

        systemB.whenPressed(new SequentialCommandGroup(
                new LimelightCommands.AlignXCmd(limeLightSubsystem, mecanumDrive),
                new SetStateCommands.IntakeStateCmd()));
        systemX.whenPressed(new SequentialCommandGroup(
                new LimelightCommands.LimelightStartIntake(limeLightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive, Pipelines.BLUE),
                new SetStateCommands.IntakeStateCmd()));

        systemDPadUp.whenPressed(new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem));
        systemDPadLeft.whenPressed(new DischargeCommands.HpDischarge(dischargeSubsystem));


        systemBack.whenPressed(new IntakeCommands.SlideHomeCmd(intakeSubsystem, false));

        systemLeftBumper.whenPressed(new SequentialCommandGroup(
                new SetStateCommands.NoneStateCmd(),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)));

        systemDPadDown.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0)));

    }
}
