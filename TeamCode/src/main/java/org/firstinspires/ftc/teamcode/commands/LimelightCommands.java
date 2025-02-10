package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

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
//            currentPipeline = limelight.
        }

        @Override
        public void execute() {
            mecanumDrive.drive((limelight.getXDistance() - limelight.middleOfScreen) * kp, 0, 0, 0.2);
        }
    }
}
