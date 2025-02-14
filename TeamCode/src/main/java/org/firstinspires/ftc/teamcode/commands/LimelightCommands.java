package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class LimelightCommands {
    public static class AlignXCmd extends CommandBase {
        LimelightSubsystem limelight;
        MecanumDrive mecanumDrive;
        final double kp = 0.005;
        double currentPipeline;

        public AlignXCmd(LimelightSubsystem limelight, MecanumDrive mecanumDrive) {
            this.limelight = limelight;
            this.mecanumDrive = mecanumDrive;
        }

        @Override
        public void initialize() {
            limelight.startLimelight();
            mecanumDrive.setFieldOriented(false);
        }

        @Override
        public void execute() {
            mecanumDrive.drive(limelight.getXDistance() * kp + 0.05, 0, 0, 0.2);
        }

        @Override
        public boolean isFinished() {
            return limelight.getXDistance() <= 15;
        }

        @Override
        public void end(boolean interrupted) {
            mecanumDrive.setFieldOriented(true);
            mecanumDrive.drive(0, 0, 0, 0);
            limelight.alignedY = limelight.getYDistance();
            limelight.stopLimelight();
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
            addCommands(new AlignXCmd(limelightSubsystem, mecanumDrive).withTimeout(1000),
                    new WaitCommand(100),
                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, limelightSubsystem::getYDistance),
//                    new WaitCommand(1000),
                    new IntakeCommands.SetRotationCmd(intakeSubsystem, limelightSubsystem::getAngle),
                    new WaitCommand(500),
                    new IntakeCommands.SampleReverseIntakeCmd(intakeSubsystem).withTimeout(500),
                    new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem),
                    new WaitCommand(500),
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
