package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Pipelines;

public class LimelightCommands {
    public static class AlignXCmd extends CommandBase {
        LimelightSubsystem limelight;
        MecanumDrive mecanumDrive;
        final double kp = 0.01;
        double currentPipeline;

        public AlignXCmd(LimelightSubsystem limelight) {
            this.limelight = limelight;
        }

        @Override
        public void initialize() {
            limelight.startLimelight();
        }

        @Override
        public void execute() {
            mecanumDrive.drive(limelight.getXDistance() * kp, 0, 0, 0.2);
        }

        @Override
        public boolean isFinished() {
            return limelight.getXDistance() <= 1;
        }

        @Override
        public void end(boolean interrupted) {
            limelight.stopLimelight();
        }
    }

    public static class LimelightIntake extends SequentialCommandGroup {
        LimelightSubsystem limelightSubsystem;
        IntakeSubsystem intakeSubsystem;
        DischargeSubsystem dischargeSubsystem;

        public LimelightIntake(LimelightSubsystem limelightSubsystem, IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            addCommands(new AlignXCmd(limelightSubsystem),
                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, false, (int) limelightSubsystem.getYDistance()),
                    new IntakeCommands.SampleIntakeCmd(intakeSubsystem),
                    new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem));
        }

        @Override
        public void initialize() {
            limelightSubsystem.startLimelight();
        }
    }

}
