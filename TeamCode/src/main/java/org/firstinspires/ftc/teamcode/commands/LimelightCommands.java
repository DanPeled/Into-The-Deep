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
            limelight.startLimelight();
            mecanumDrive.setFieldOriented(false);
        }

        @Override
        public void execute() {
            mecanumDrive.drive(limelight.getXDistance() * kp + Math.signum(limelight.getXDistance()) * minPower, 0, 0, 0.5);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(limelight.getXDistance()) <= 15;
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
                    new InstantCommand(() -> {
                        while (limelightSubsystem.getYDistance() >= 1700) {
                            addCommands(new InstantCommand(() -> mecanumDrive.drive(-0.3, 0, 0, 0.2)).withTimeout(300));
                        }
                    }),
                    new AlignXCmd(limelightSubsystem, mecanumDrive)/*.withTimeout(1000)*/,
                    new WaitCommand(100),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, limelightSubsystem::getYDistance),
                                    new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.MIDDLE)),
//                    new WaitCommand(1000),
                            new SequentialCommandGroup(
                                    new WaitCommand(1000),
                                    new IntakeCommands.SetRotationCmd(intakeSubsystem, limelightSubsystem::getAngle))),
                    new WaitCommand(500),
//                    new IntakeCommands.SampleReverseIntakeCmd(intakeSubsystem).withTimeout(2000),
                    new ParallelCommandGroup(
                            new IntakeCommands.SpinCmd(intakeSubsystem, -1, 1),
                            new MecanumCommands.MecanumShakeCmd(mecanumDrive, 0.08, 0.1).withTimeout(1000),
                            new IntakeCommands.IntakeShakeCmd(intakeSubsystem, 0.4, 0.1).withTimeout(1000)),
                    new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem),
                    new WaitCommand(500),
                    new IntakeCommands.SlideUntilCmd(intakeSubsystem, 1200, 1, true),
                    new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem));
            addRequirements(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive);
        }

        public LimelightCompleteSubIntake(LimelightSubsystem limelightSubsystem, IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem, MecanumDrive mecanumDrive, Pipelines pipelines) {
            this.intakeSubsystem = intakeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            this.dischargeSubsystem = dischargeSubsystem;
            this.mecanumDrive = mecanumDrive;
            limelightSubsystem.setPipeline(pipelines);
            addCommands(
                    new InstantCommand(() -> {
                        while (limelightSubsystem.getYDistance() >= 1700) {
                            addCommands(new InstantCommand(() -> mecanumDrive.drive(-0.3, 0, 0, 0.2)).withTimeout(300));
                        }
                    }),
                    new AlignXCmd(limelightSubsystem, mecanumDrive)/*.withTimeout(1000)*/,
                    new WaitCommand(100),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, limelightSubsystem::getYDistance),
                                    new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.MIDDLE)),
//                    new WaitCommand(1000),
                            new SequentialCommandGroup(
                                    new WaitCommand(1000),
                                    new IntakeCommands.SetRotationCmd(intakeSubsystem, limelightSubsystem::getAngle))),
                    new WaitCommand(500),
//                    new IntakeCommands.SampleReverseIntakeCmd(intakeSubsystem).withTimeout(2000),
                    new ParallelCommandGroup(
                            new IntakeCommands.SpinCmd(intakeSubsystem, -1, 1),
                            new MecanumCommands.MecanumShakeCmd(mecanumDrive, 0.08, 0.1).withTimeout(1000),
                            new IntakeCommands.IntakeShakeCmd(intakeSubsystem, 0.4, 0.1).withTimeout(1000)),
                    new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem),
                    new WaitCommand(500),
                    new IntakeCommands.SlideUntilCmd(intakeSubsystem, 1200, 1, true),
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
                    new AlignXCmd(limelightSubsystem, mecanumDrive),
                    new WaitCommand(100),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, limelightSubsystem::getYDistance),
                                    new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.MIDDLE)),
//                    new WaitCommand(1000),
                            new SequentialCommandGroup(
                                    new WaitCommand(500),
                                    new IntakeCommands.SetRotationCmd(intakeSubsystem, limelightSubsystem::getAngle))),
                    new WaitCommand(500),
//                    new IntakeCommands.SampleReverseIntakeCmd(intakeSubsystem).withTimeout(2000),
//                    new ParallelCommandGroup(
//                            new IntakeCommands.SpinCmd(intakeSubsystem, -1, 1),
//                            new MecanumCommands.MecanumShakeCmd(mecanumDrive, 0.08, 0.1).withTimeout(1000),
//                            new IntakeCommands.IntakeShakeCmd(intakeSubsystem, 0.4, 0.1).withTimeout(1000)),
                    new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem),
                    new InstantCommand(new Runnable() {
                        @Override
                        public void run() {
                            intakeSubsystem.setArmPower(0);
                            intakeSubsystem.runWithoutEncoders();
                        }
                    })
//                    new WaitCommand(500),
//                    new IntakeCommands.SlideUntilCmd(intakeSubsystem, 1200, 1, true),
//                    new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem)
            );
            addRequirements(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive);
        }

        public LimelightStartIntake(LimelightSubsystem limelightSubsystem, IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem, MecanumDrive mecanumDrive, Pipelines pipelines) {
            this.intakeSubsystem = intakeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            this.dischargeSubsystem = dischargeSubsystem;
            this.mecanumDrive = mecanumDrive;
            limelightSubsystem.setPipeline(pipelines);
            addCommands(
//                    new InstantCommand(() -> {
//                        while (limelightSubsystem.getYDistance() >= 1700) {
//                            addCommands(new InstantCommand(() -> mecanumDrive.drive(-0.3, 0, 0, 0.2)).withTimeout(300));
//                        }
//                    }),
                    new AlignXCmd(limelightSubsystem, mecanumDrive),
                    new WaitCommand(100),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, limelightSubsystem::getYDistance),
                                    new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.MIDDLE)),
//                    new WaitCommand(1000),
                            new SequentialCommandGroup(
                                    new WaitCommand(500),
                                    new IntakeCommands.SetRotationCmd(intakeSubsystem, limelightSubsystem::getAngle))),
                    new WaitCommand(500),
//                    new IntakeCommands.SampleReverseIntakeCmd(intakeSubsystem).withTimeout(2000),
//                    new ParallelCommandGroup(
//                            new IntakeCommands.SpinCmd(intakeSubsystem, -1, 1),
//                            new MecanumCommands.MecanumShakeCmd(mecanumDrive, 0.08, 0.1).withTimeout(1000),
//                            new IntakeCommands.IntakeShakeCmd(intakeSubsystem, 0.4, 0.1).withTimeout(1000)),
                    new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem)
//                    new WaitCommand(500),
//                    new IntakeCommands.SlideUntilCmd(intakeSubsystem, 1200, 1, true),
//                    new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem)
            );
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
