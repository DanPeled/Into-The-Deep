package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargeGrabCmd;

import java.util.function.Supplier;

public class IntakeCommands {

    public static class NoOpCommand extends CommandBase {
        public NoOpCommand(IntakeSubsystem intakeSubsystem) {
            addRequirements(intakeSubsystem);
        }

        @Override
        public void execute() {
            // Do nothing
        }

        @Override
        public boolean isFinished() {
            return false; // Runs indefinitely
        }
    }

    public static class SlideGotoCmd extends CommandBase {
        IntakeSubsystem subsystem;
        final int position;
        final int maxPosition = 3000;

        public SlideGotoCmd(IntakeSubsystem subsystem, int position) {
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
            subsystem.runWithEncoders();
            subsystem.setTargetPos(position);
            subsystem.armGoToTarget();
        }

        @Override
        public boolean isFinished() {
            return Math.abs(subsystem.getMotorPosition() - position) <= 20;
        }
    }

    public static class SlideHomeCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        double lastTick;
        double lastTime = 0;
        final double maxDuration = 2.5;
        final boolean initTime;
        final int minPosOffset = 40;
        ElapsedTime elapsedTime = new ElapsedTime();

        public SlideHomeCmd(IntakeSubsystem intakeSubsystem, boolean initTime) {
            this.intakeSubsystem = intakeSubsystem;
            this.initTime = initTime;
            //addRequirements(intakeSubsystem);

        }

        @Override
        public void initialize() {
            lastTick = intakeSubsystem.getAveragePosition();
            elapsedTime.reset();
            intakeSubsystem.setArmPower(0);
            intakeSubsystem.runWithoutEncoders();
            addRequirements(intakeSubsystem);
        }

        @Override
        public void execute() {
            if (initTime) {
                intakeSubsystem.setRawPower(-intakeSubsystem.slidesLowSpeed);
            } else if (intakeSubsystem.getMotorPosition() > 225) {
                intakeSubsystem.setRawPower(-intakeSubsystem.slidesSpeed);
            } else {
                intakeSubsystem.setRawPower(-intakeSubsystem.slidesLowSpeed);
            }

        }

        @Override
        public boolean isFinished() {

            if (!initTime && elapsedTime.seconds() > maxDuration) {
                return true;
            }

            double deltaTime = elapsedTime.seconds() - lastTime;
            if (deltaTime > 0.2) {
                double deltaTick = intakeSubsystem.getAveragePosition() - lastTick;
                lastTick = intakeSubsystem.getAveragePosition();
                lastTime = elapsedTime.seconds();
                if (Math.abs(deltaTick) <= 8)
                    return true;

            }

            return false;
        }

        @Override
        public void end(boolean interrupted) {
            intakeSubsystem.setArmPower(0);

            if (!interrupted) {
                intakeSubsystem.resetEncoders();
                intakeSubsystem.setTargetPos(intakeSubsystem.getMotorPosition() + minPosOffset);
                intakeSubsystem.minSlidesPos = intakeSubsystem.getMotorPosition() + minPosOffset;
            }

        }
    }

    public static class ClawStageCmd extends CommandBase {
        IntakeSubsystem subsystem;
        double stage;

        public ClawStageCmd(IntakeSubsystem subsystem, double stage) {
            this.stage = stage;
            this.subsystem = subsystem;
//            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setHServoPosition(stage);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    //doesn't have addRequirements(subsystem);
    public static class SetArmsStageCmd extends CommandBase {
        IntakeSubsystem subsystem;
        double armsStage;

        public SetArmsStageCmd(IntakeSubsystem subsystem, double armsStage) {
            this.subsystem = subsystem;
            this.armsStage = armsStage;
//            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setArmsStage(armsStage);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class SetRotationCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        double position;

        public SetRotationCmd(IntakeSubsystem intakeSubsystem, double position) {
            this.intakeSubsystem = intakeSubsystem;
            this.position = position;
//            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setRotationServoPosition(position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    //doesn't have addRequirements
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
            intakeSubsystem.setRotationServoPosition(position.get());
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    //doesn't have addRequirements
    public static class IntakeManualGoToCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        Supplier<Double> power;
        ElapsedTime elapsedTime = new ElapsedTime();
        double lastTimeMilli = 0;

        public IntakeManualGoToCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> power) {
            this.intakeSubsystem = intakeSubsystem;
            this.power = power;

            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize(){
            elapsedTime.reset();
        }

        @Override
        public void execute() {
            double timeMilli = elapsedTime.milliseconds();
            if (Math.abs(power.get()) > 0.2) {
                intakeSubsystem.runWithEncoders();
                intakeSubsystem.changeTargetPos(power.get() * ((timeMilli - lastTimeMilli)*(intakeSubsystem.manualTicksPerSecond/1000.0)));
                intakeSubsystem.armGoToTarget();
            }
            lastTimeMilli = timeMilli;
        }

    }

    public static class SpinCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        ElapsedTime runtime = new ElapsedTime();
        final double duration, power;
        double startTime;

        //power from -1 to 1
        public SpinCmd(IntakeSubsystem intakeSubsystem, double power, double duration) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
            this.duration = duration;
            this.power = power;
        }

        @Override
        public void initialize() {
            intakeSubsystem.setSpinPower(power);
            startTime = runtime.seconds();
        }

        @Override
        public boolean isFinished() {
            return duration <= 0 || (runtime.seconds() - startTime) > duration;
        }

        @Override
        public void end(boolean interrupted) {
            if (duration != -1)
                intakeSubsystem.setSpinPower(0);
        }
    }

    public static class Wait extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        ElapsedTime runtime = new ElapsedTime();
        final double duration;
        double startTime;

        //power from -1 to 1
        public Wait(IntakeSubsystem intakeSubsystem, double duration) {
            this.intakeSubsystem = intakeSubsystem;
            this.duration = duration;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            startTime = runtime.seconds();
        }

        @Override
        public boolean isFinished() {
            return (runtime.seconds() - startTime) > duration;
        }
    }

    //if doesnt work check if everything needs to not have addRequirements(subsystem);
    public static class StartIntakeCmd extends SequentialCommandGroup {
        private final int pos = 1700;

        public StartIntakeCmd(IntakeSubsystem subsystem) {
            addCommands(
                    new SpinCmd(subsystem, 0, 0),
                    new SetArmsStageCmd(subsystem, ArmsStages.BOTTOM),
                    new SetRotationCmd(subsystem, 0.5),
                    new SlideGotoCmd(subsystem, pos),
                    new ClawStageCmd(subsystem, ClawStages.LOWER),
                    new Wait(subsystem, 0.25),
                    new SetArmsStageCmd(subsystem, ArmsStages.TOP));
            addRequirements(subsystem);
        }

    }

    public static class reStartIntakeCmd extends SequentialCommandGroup {
        public reStartIntakeCmd(IntakeSubsystem subsystem) {
            addCommands(
                    new SpinCmd(subsystem, 0, 0),
                    new SetArmsStageCmd(subsystem, ArmsStages.TOP));
            addRequirements(subsystem);
        }

    }

    public static class ManualIntakeCmd extends ParallelCommandGroup {
        public ManualIntakeCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> power, Supplier<Double> position) {
            addCommands
                    (new SetZRotationSupplierCmd(intakeSubsystem, position),
                            new IntakeManualGoToCmd(intakeSubsystem, power));
            addRequirements(intakeSubsystem);
        }
    }

    public static class SampleIntakeCmd extends SequentialCommandGroup {
        public SampleIntakeCmd(IntakeSubsystem intakeSubsystem) {
            final double
                    spinPower = 1,
                    middleTime = 0.75,
                    grabbingTime = 0.5,
                    holdingPower = 0.05;


            addCommands(
                    new ParallelCommandGroup(
                            new SetArmsStageCmd(intakeSubsystem, ArmsStages.MIDDLE),
                            new SpinCmd(intakeSubsystem, -spinPower, middleTime)),
                    new ParallelCommandGroup(
                            new SetArmsStageCmd(intakeSubsystem, ArmsStages.BOTTOM),
                            new SpinCmd(intakeSubsystem, spinPower, grabbingTime)),
                    new SpinCmd(intakeSubsystem, holdingPower, -1));
        }
    }

    public static class ReturnArmCmd extends SequentialCommandGroup {
        public ReturnArmCmd(IntakeSubsystem intakeSubsystem, boolean initTime) {
            addCommands(
                    new SetArmsStageCmd(intakeSubsystem, ArmsStages.BOTTOM),
                    new SetRotationCmd(intakeSubsystem, 0.5),
                    //new Wait(intakeSubsystem, 0.0),
                    new ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
                    new Wait(intakeSubsystem, 1),
                    new ClawStageCmd(intakeSubsystem, ClawStages.UPPER),//for safety
                    new SlideHomeCmd(intakeSubsystem, initTime));
            addRequirements(intakeSubsystem);
        }
    }

    public static class Transfer extends SequentialCommandGroup {
        final int slidesBackAfterTransfer = 10;
        public Transfer(IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem) {
            addCommands(
                    new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                    new ParallelCommandGroup(
                        new DischargeCommands.GoHomeCmd(dischargeSubsystem),
                        new ReturnArmCmd(intakeSubsystem, false)),
                    new DischargeGrabCmd(dischargeSubsystem),
                    new SetArmsStageCmd(intakeSubsystem, ArmsStages.TRANSFER),
                    new SpinCmd(intakeSubsystem, 0, -1),
                    new Wait(intakeSubsystem,0.75),
                    new SlideGotoCmd(intakeSubsystem, intakeSubsystem.minSlidesPos+slidesBackAfterTransfer));

            addRequirements(intakeSubsystem, dischargeSubsystem); //may be unnecessary
        }


    }

}