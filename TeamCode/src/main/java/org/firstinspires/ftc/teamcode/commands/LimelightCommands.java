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
        final double kp = 0.05;
        double currentPipeline;

        public AlignXCmd(LimelightSubsystem limelight, MecanumDrive mecanumDrive) {
            this.limelight = limelight;
            this.mecanumDrive = mecanumDrive;
        }

        @Override
        public void initialize() {
//            limelight.startLimelight();
            mecanumDrive.setFieldOriented(false);
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
            mecanumDrive.setFieldOriented(true);
        }
    }

    public static class LimelightIntake extends SequentialCommandGroup {
        LimelightSubsystem limelightSubsystem;
        IntakeSubsystem intakeSubsystem;
        DischargeSubsystem dischargeSubsystem;
        MecanumDrive mecanumDrive;
        double wantedAngle;

        public LimelightIntake(LimelightSubsystem limelightSubsystem, IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem, MecanumDrive mecanumDrive) {
            this.intakeSubsystem = intakeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            this.dischargeSubsystem = dischargeSubsystem;
            this.mecanumDrive = mecanumDrive;
            addCommands(new AlignXCmd(limelightSubsystem, mecanumDrive),
                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, (int) limelightSubsystem.getYDistance()),
                    new IntakeCommands.SetRotationCmd(intakeSubsystem, wantedAngle),
                    new IntakeCommands.SampleIntakeCmd(intakeSubsystem),
                    new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem));
            addRequirements(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive);
        }

        @Override
        public void initialize() {
            super.initialize();
//            limelightSubsystem.startLimelight();
            wantedAngle = limelightSubsystem.getAngle();
        }

        @Override
        public void end(boolean interrupted) {
//            limelightSubsystem.stopLimelight();
        }
    }

}
