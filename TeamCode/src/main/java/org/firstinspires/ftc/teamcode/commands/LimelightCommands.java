package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Pipelines;

public class LimelightCommands {
    @Config
    public static class AlignXCmd extends CommandBase {
        LimelightSubsystem limelight;
        MecanumDrive mecanumDrive;
        public static double kp = 0.0008;
        public static double minPower = 0.1;
        double currentPipeline;

        public AlignXCmd(LimelightSubsystem limelight, MecanumDrive mecanumDrive) {
            this.limelight = limelight;
            this.mecanumDrive = mecanumDrive;
            addRequirements(mecanumDrive, limelight);
        }

        @Override
        public void initialize() {
//            limelight.startLimelight();
            limelight.updateResults();
            mecanumDrive.setFieldOriented(false);
        }

        @Override
        public void execute() {
            mecanumDrive.drive(limelight.getXDistance() * kp + Math.signum(limelight.getXDistance()) * minPower, 0, 0, 0.5);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(limelight.getXDistance()) <= 5;
        }

        @Override
        public void end(boolean interrupted) {
            mecanumDrive.setFieldOriented(true);
            mecanumDrive.drive(0, 0, 0, 0);
            limelight.alignedY = limelight.getYDistance();
            limelight.stopLimelight();
        }
    }

    public static class LimelightCompleteSubIntake extends SequentialCommandGroup {
        LimelightSubsystem limelightSubsystem;
        IntakeSubsystem intakeSubsystem;
        DischargeSubsystem dischargeSubsystem;
        MecanumDrive mecanumDrive;
        double wantedAngle;
        boolean outOfRange = false;

        public LimelightCompleteSubIntake(LimelightSubsystem limelightSubsystem, IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem, MecanumDrive mecanumDrive) {
            this.intakeSubsystem = intakeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            this.dischargeSubsystem = dischargeSubsystem;
            this.mecanumDrive = mecanumDrive;
            addCommands(
//                    new InstantCommand(() -> {
//                        while (limelightSubsystem.getYDistance() >= 1700) {
//                            addCommands(new InstantCommand(() -> mecanumDrive.drive(-0.3, 0, 0, 0.2)).withTimeout(300));
//                        }
//                    }),
                    new AlignXCmd(limelightSubsystem, mecanumDrive)/*.withTimeout(1000)*/,
                    new WaitCommand(100),
                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, limelightSubsystem::getYDistance),
                    new ParallelCommandGroup(
                            new IntakeCommands.SetRotationCmd(intakeSubsystem, limelightSubsystem::getAngle),
                            new IntakeCommands.OpenScrewCmd(intakeSubsystem, true)
                    ),
                    new WaitCommand(500),
                    new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem));
            addRequirements(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive);
        }


        @Override
        public void initialize() {
            super.initialize();
            limelightSubsystem.startLimelight();
            wantedAngle = limelightSubsystem.getAngle();
        }

        @Override
        public void end(boolean interrupted) {
            limelightSubsystem.stopLimelight();
        }
    }

    public static class LimelightStartIntake extends SequentialCommandGroup {
        LimelightSubsystem limelightSubsystem;
        IntakeSubsystem intakeSubsystem;
        DischargeSubsystem dischargeSubsystem;
        MecanumDrive mecanumDrive;
        double wantedAngle;
        boolean outOfRange = false;

        public LimelightStartIntake(LimelightSubsystem limelightSubsystem, IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem, MecanumDrive mecanumDrive) {
            this.intakeSubsystem = intakeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            this.dischargeSubsystem = dischargeSubsystem;
            this.mecanumDrive = mecanumDrive;
            addCommands(
//                    new InstantCommand(() -> {
//                        while (limelightSubsystem.getYDistance() >= 1700) {
//                            addCommands(new InstantCommand(() -> mecanumDrive.drive(-0.3, 0, 0, 0.2)).withTimeout(300));
//                        }
//                    }),
                    new AlignXCmd(limelightSubsystem, mecanumDrive)/*.withTimeout(1000)*/,
                    new WaitCommand(100),
                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, limelightSubsystem::getYDistance),
                    new ParallelCommandGroup(
                            new IntakeCommands.SetRotationCmd(intakeSubsystem, limelightSubsystem::getAngle),
                            new IntakeCommands.OpenScrewCmd(intakeSubsystem, true)
                    ),
                    new WaitCommand(500),

                    new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),

                    new InstantCommand(new Runnable() {
                        @Override
                        public void run() {
                            intakeSubsystem.setArmPower(0);
                            intakeSubsystem.runWithoutEncoders();
                        }
                    }));

            addRequirements(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive);
        }


        @Override
        public void initialize() {
            super.initialize();
            limelightSubsystem.startLimelight();
            wantedAngle = limelightSubsystem.getAngle();
        }

        @Override
        public void end(boolean interrupted) {
            limelightSubsystem.stopLimelight();
        }
    }

}
