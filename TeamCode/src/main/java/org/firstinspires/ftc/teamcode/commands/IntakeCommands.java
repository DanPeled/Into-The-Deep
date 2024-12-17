package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripStages;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.XRotationStages;

import java.util.function.Supplier;

public class IntakeCommands {
    public static class IntakeGotoCmd extends CommandBase {
        IntakeSubsystem subsystem;
        final int position;
        final int maxPosition = 400;
        final double kp = 0.01;

        public IntakeGotoCmd(IntakeSubsystem subsystem, int position) {
            this.subsystem = subsystem;
            if (position > maxPosition) {
                position = maxPosition;
            } else if (position < 0) {
                position = 0;
            }
            this.position = position;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.armGoToPos(position);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(position - subsystem.getMotorPosition()) <= 5;
        }

        @Override
        public void end(boolean interrupted) {
            subsystem.setArmPower(0);
        }
    }

    public static class SetXRotationCmd extends CommandBase {
        IntakeSubsystem subsystem;
        XRotationStages stage;

        public SetXRotationCmd(IntakeSubsystem subsystem, XRotationStages stage) {
            this.stage = stage;
            this.subsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setXServoPosition(stage.POSITION);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(subsystem.getXServoPosition() - stage.POSITION) <= 0.02;
        }
    }

    //doesnt have addRequirements(subsystem);
    public static class SetGripStageCmd extends CommandBase {
        IntakeSubsystem subsystem;
        GripStages gripStage;

        public SetGripStageCmd(IntakeSubsystem subsystem, GripStages gripStage) {
            this.subsystem = subsystem;
            this.gripStage = gripStage;
//            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setGripStage(gripStage);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(subsystem.getGripServoPosition() - gripStage.POSITION) < 0.02;
        }
    }

    public static class SetZRotationCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        double position;

        public SetZRotationCmd(IntakeSubsystem intakeSubsystem, double position) {
            this.intakeSubsystem = intakeSubsystem;
            this.position = position;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setZServoPosition(position);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(intakeSubsystem.getZServoPosition() - position) < 0.02;
        }
    }

    //doesnt have addRequirements
    public static class SetZRotationSupplierCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        Supplier<Double> position;

        public SetZRotationSupplierCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> position) {
            this.intakeSubsystem = intakeSubsystem;
            this.position = position;
//            addRequirements(intakeSubsystem);
        }

        @Override
        public void execute() {
            intakeSubsystem.setZServoPosition(position.get());
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    //doesnt have addRequirements
    public static class SetPowerSupplierCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        Supplier<Double> power;

        public SetPowerSupplierCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> power) {
            this.intakeSubsystem = intakeSubsystem;
            this.power = power;

//            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.powerMode();
        }

        @Override
        public void execute() {
            intakeSubsystem.setArmPower(power.get());
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public static class SpinOutCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        ElapsedTime runtime;
        double startTime, duration;

        public SpinOutCmd(IntakeSubsystem intakeSubsystem, ElapsedTime runtime, double duration) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
            this.duration = duration;
        }

        @Override
        public void initialize() {
            intakeSubsystem.setSpinPower(-0.5);
            startTime = runtime.seconds();
        }

        @Override
        public boolean isFinished() {
            return (runtime.seconds() - startTime) > duration;
        }

        @Override
        public void end(boolean interrupted) {
            intakeSubsystem.setSpinPower(0);
        }
    }

    public static class SpinInCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        ElapsedTime runtime;
        double startTime, duration;

        public SpinInCmd(IntakeSubsystem intakeSubsystem, ElapsedTime runtime, double duration) {
            this.intakeSubsystem = intakeSubsystem;
            this.duration = duration;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setSpinPower(0.5);
            startTime = runtime.seconds();
        }

        @Override
        public boolean isFinished() {
            return (runtime.seconds() - startTime) > duration;
        }

        @Override
        public void end(boolean interrupted) {
            intakeSubsystem.setSpinPower(0);
        }
    }

    //if doesnt work check if everything needs to not have addRequirements(subsystem);
    public static class StartIntakeCmd extends SequentialCommandGroup {
        private final int pos = 500;

        public StartIntakeCmd(IntakeSubsystem subsystem) {
            addCommands(new IntakeGotoCmd(subsystem, pos),
                    new SetXRotationCmd(subsystem, XRotationStages.MIDDLE),
                    new ParallelCommandGroup(new SetGripStageCmd(subsystem, GripStages.TOP),
                            new SetXRotationCmd(subsystem, XRotationStages.LOWER)));
            addRequirements(subsystem);
        }

    }

    public static class ManualIntakeCmd extends ParallelCommandGroup {
        public ManualIntakeCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> power, Supplier<Double> position) {
            addCommands(new SetZRotationSupplierCmd(intakeSubsystem, position),
                    new SetPowerSupplierCmd(intakeSubsystem, power));
            addRequirements(intakeSubsystem);
        }
    }

    public static class SampleIntakeCmd extends SequentialCommandGroup {
        public SampleIntakeCmd(IntakeSubsystem intakeSubsystem, ElapsedTime runtime) {
            addCommands(new ParallelCommandGroup(new SetGripStageCmd(intakeSubsystem, GripStages.MIDDLE),
                            new SpinOutCmd(intakeSubsystem, runtime, 0.2)),
                    new ParallelCommandGroup(new SetGripStageCmd(intakeSubsystem, GripStages.BOTTOM),
                            new SpinInCmd(intakeSubsystem, runtime, 0.2)));
        }
    }

    public static class ReturnArmCmd extends SequentialCommandGroup {
        public ReturnArmCmd(IntakeSubsystem intakeSubsystem) {
            addCommands(new ParallelCommandGroup(new SetGripStageCmd(intakeSubsystem, GripStages.BOTTOM),
                            new SetZRotationCmd(intakeSubsystem, 0)),
                    new SetXRotationCmd(intakeSubsystem, XRotationStages.UPPER),
                    new IntakeGotoCmd(intakeSubsystem, 0));
        }
    }
}
