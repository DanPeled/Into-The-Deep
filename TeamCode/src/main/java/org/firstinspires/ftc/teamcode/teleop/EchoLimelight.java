package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystems.Pipelines;


@TeleOp
public class EchoLimelight extends Echo {
    @Override
    public void run() {
        super.run();
        limeLightSubsystem.updateResults();
    }

    @Override
    public void noneBindings() {
//        systemDPadUp.whenPressed(new LimelightCommands.LimelightCompleteSubIntake(limeLightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive));
//        systemDPadDown.whenPressed(new LimelightCommands.LimelightStartIntake(limeLightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive));
        systemDPadLeft.whenPressed(new LimelightCommands.AlignXCmd(limeLightSubsystem, mecanumDrive));
        systemX.whenPressed(() -> limeLightSubsystem.setPipeline(Pipelines.BLUE));
        systemY.whenPressed(() -> limeLightSubsystem.setPipeline(Pipelines.YELLOW));
        systemB.whenPressed(() -> limeLightSubsystem.setPipeline(Pipelines.RED));
        systemA.whenPressed(() -> limeLightSubsystem.limelight.reloadPipeline());
    }

    @Override
    public void telemetries() {
        telemetry.addData("runnig", limeLightSubsystem.limelight.isRunning());
        telemetry.addData("cinfo", limeLightSubsystem.limelight.getConnectionInfo());
        super.telemetries();
    }
}
