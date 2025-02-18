package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargeGrabCmd;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

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
        int position = 0;
        Supplier<Integer> positionSupplier;
        boolean limelight = false;
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

        public SlideGotoCmd(IntakeSubsystem subsystem, Supplier<Integer> position) {
            this.subsystem = subsystem;
            positionSupplier = position;
            limelight = true;
        }

        @Override
        public void initialize() {
            if (limelight) {
                subsystem.setTargetPos(positionSupplier.get());
            } else {
                subsystem.setTargetPos(position);
            }
            subsystem.armGoToTarget();
            subsystem.runWithEncoders();
        }

        @Override
        public boolean isFinished() {
            if (limelight) {
                return Math.abs(subsystem.getMotorPosition() - positionSupplier.get()) <= 15;
            }
            return Math.abs(subsystem.getMotorPosition() - position) <= 15;
        }
    }

    public static class SlideHomeCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        double lastTick;
        double lastTime = 0;
        final double maxDuration = 3;
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
                intakeSubsystem.setRawPower(-0.4);
            } else if (intakeSubsystem.getMotorPosition() > 60) {
                intakeSubsystem.setRawPower(-intakeSubsystem.slidesSpeed);
            } else {
//                intakeSubsystem.setRawPower(-intakeSubsystem.slidesLowSpeed);
                intakeSubsystem.setRawPower(-intakeSubsystem.slidesLowSpeed);
            }

        }

        @Override
        public boolean isFinished() {
            if (!initTime && elapsedTime.seconds() > maxDuration) {
                return true;
            }

//            double deltaTime = elapsedTime.seconds() - lastTime;
//            if (deltaTime > 0.2) {
//                double avg = intakeSubsystem.getAveragePosition();
//                double deltaTick = avg - lastTick;
//                lastTick = avg;
//                lastTime = elapsedTime.seconds();
//                return (Math.abs(deltaTick) <= 8 && (avg < 200 || initTime));

//            }
            return intakeSubsystem.isHome() || (intakeSubsystem.getCurrent() > 2.5 && intakeSubsystem.getAveragePosition() < 1000);
        }

        @Override
        public void end(boolean interrupted) {
            intakeSubsystem.resetEncoders();
        }
    }

    public static class SlideHomeTouchCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        double lastTick;
        double lastTime = 0;
        final double maxDuration = 3;
        final boolean initTime;
        final int minPosOffset = 40;
        ElapsedTime elapsedTime = new ElapsedTime();

        public SlideHomeTouchCmd(IntakeSubsystem intakeSubsystem, boolean initTime) {
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
                intakeSubsystem.setRawPower(-0.2);
            } else {
                intakeSubsystem.setRawPower(-1);
            }

        }

        @Override
        public boolean isFinished() {


            return intakeSubsystem.isHome() || (!initTime && elapsedTime.seconds() > maxDuration);

        }

        @Override
        public void end(boolean interrupted) {
            intakeSubsystem.setArmPower(0);
            intakeSubsystem.end = true;
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
        ElapsedTime elapsedTime = new ElapsedTime();

        public ClawStageCmd(IntakeSubsystem subsystem, double stage) {
            this.stage = stage;
            this.subsystem = subsystem;
//            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setHServoPosition(stage);
            elapsedTime.reset();
        }

        @Override
        public boolean isFinished() {
            return elapsedTime.seconds() > 0.0;
        }
    }

    //doesn't have addRequirements(subsystem);
    public static class SetArmsStageCmd extends CommandBase {
        IntakeSubsystem subsystem;
        double armsStage;
        ElapsedTime elapsedTime = new ElapsedTime();

        public SetArmsStageCmd(IntakeSubsystem subsystem, double armsStage) {
            this.subsystem = subsystem;
            this.armsStage = armsStage;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            elapsedTime.reset();
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
        Supplier<Double> positionSupplier;
        boolean limelight = false;

        public SetRotationCmd(IntakeSubsystem intakeSubsystem, double position) {
            this.intakeSubsystem = intakeSubsystem;
            this.position = position;
//            addRequirements(intakeSubsystem);
        }

        public SetRotationCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> position) {
            this.intakeSubsystem = intakeSubsystem;
            this.positionSupplier = position;
            limelight = true;
//            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            if (limelight)
                intakeSubsystem.setRotationServoPosition((1 - (positionSupplier.get() + 90) / 180 - 0.5) * 2 / 3 + 0.5);
            else
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

    public static class IntakeManualGoToCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        Supplier<Double> power;
        ElapsedTime elapsedTime = new ElapsedTime();
        double lastTimeMilli = 0;
        private static boolean isEnabled = true;
        private static IntakeManualGoToCmd currentInstance;

        public IntakeManualGoToCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> power) {
            this.intakeSubsystem = intakeSubsystem;
            this.power = power;


            currentInstance = this;
            addRequirements(intakeSubsystem);
        }

        public static void setEnabled(boolean enabled) {
            isEnabled = enabled;
        }

        public static boolean isEnabled() {
            return isEnabled;
        }

        @Override
        public void initialize() {
            elapsedTime.reset();
        }

        @Override
        public void execute() {
            if (isEnabled) {
                intakeSubsystem.setArmPower(power.get() * 0.5);
            }
//            if (isEnabled) {
//                double timeMilli = elapsedTime.milliseconds();
//                if (Math.abs(power.get()) > 0.2) {
//                    intakeSubsystem.runWithEncoders();
//                    intakeSubsystem.changeTargetPos(power.get() * ((timeMilli - lastTimeMilli) * (intakeSubsystem.manualTicksPerSecond / 1000.0)));
//                    intakeSubsystem.armGoToTarget();
//                }
//                lastTimeMilli = timeMilli;
//            }
        }


        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            currentInstance = null; // Clear the static reference when the command ends
        }


        public static void endCommand() {
            if (currentInstance != null) {
                CommandScheduler.getInstance().cancel(currentInstance);
            }
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
//            addRequirements(intakeSubsystem);
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
            return (duration <= 0) || ((runtime.seconds() - startTime) > duration);
        }

        @Override
        public void end(boolean interrupted) {
            if (duration != -1)
                intakeSubsystem.setSpinPower(0);
        }
    }

    public static class SetPowerCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        double power;

        public SetPowerCmd(IntakeSubsystem intakeSubsystem, double power) {
            this.intakeSubsystem = intakeSubsystem;
            this.power = power;
        }

        @Override
        public void initialize() {
            intakeSubsystem.setRawPower(power);
        }

        @Override
        public boolean isFinished() {
            return true;
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
        private final int pos = 1060;

        public StartIntakeCmd(IntakeSubsystem subsystem) {
            StartIntake(subsystem, false, pos);
        }

        public StartIntakeCmd(IntakeSubsystem subsystem, boolean auto) {
            StartIntake(subsystem, auto, pos);
        }

        public StartIntakeCmd(IntakeSubsystem subsystem, boolean auto, int position) {
            StartIntake(subsystem, auto, position);
        }

        public StartIntakeCmd(IntakeSubsystem subsystem, boolean auto, Supplier<Integer> position) {
//            StartIntake(subsystem, auto, position);
            addCommands(
                    new SpinCmd(subsystem, 0, 0),
                    new ClawStageCmd(subsystem, ClawStages.UPPER),
                    new SetArmsStageCmd(subsystem, ArmsStages.SHRINK),
                    new SetRotationCmd(subsystem, 0.5),
                    new SlideGotoCmd(subsystem, position),
                    new ClawStageCmd(subsystem, ClawStages.MIDDLE),
                    new SetArmsStageCmd(subsystem, ArmsStages.TOP),
                    new WaitCommand(200),
                    new ClawStageCmd(subsystem, ClawStages.LOWER));


        }


        private void StartIntake(IntakeSubsystem subsystem, boolean auto, int position) {
            if (auto) {
                addCommands(
                        new SpinCmd(subsystem, 0, 0),
                        new ClawStageCmd(subsystem, ClawStages.UPPER),
                        new SetArmsStageCmd(subsystem, ArmsStages.SHRINK),
                        new SetRotationCmd(subsystem, 0.5),
                        new SlideGotoCmd(subsystem, position),
                        new ParallelCommandGroup(
                                new SetArmsStageCmd(subsystem, ArmsStages.TOP),
                                new SequentialCommandGroup(
                                        new WaitCommand(2000),
                                        new ClawStageCmd(subsystem, ClawStages.LOWER)
                                )

                        ),
                        new WaitCommand(100)
                );
            } else {
                addCommands(
                        new SpinCmd(subsystem, 0, 0),
                        new ClawStageCmd(subsystem, ClawStages.UPPER),
                        new SetArmsStageCmd(subsystem, ArmsStages.SHRINK),
                        new SetRotationCmd(subsystem, 0.5),
                        new SlideUntilCmd(subsystem, position, 1, true),
                        new SetArmsStageCmd(subsystem, ArmsStages.TOP),
                        new ClawStageCmd(subsystem, ClawStages.LOWER),
                        new ParallelCommandGroup(
                                new SetArmsStageCmd(subsystem, ArmsStages.TOP),
                                new SequentialCommandGroup(
                                        new WaitCommand(2000),
                                        new ClawStageCmd(subsystem, ClawStages.LOWER)
                                )

                        ),
                        new WaitCommand(100)
                );
                addRequirements(subsystem);
            }
        }

        private void StartIntake(IntakeSubsystem subsystem, boolean auto, Supplier<Integer> position) {
            if (auto) {
                addCommands(
                        new SpinCmd(subsystem, 0, 0),
                        new ClawStageCmd(subsystem, ClawStages.UPPER),
                        new SetArmsStageCmd(subsystem, ArmsStages.SHRINK),
                        new SetRotationCmd(subsystem, 0.5),
                        new SlideGotoCmd(subsystem, position),
                        new ClawStageCmd(subsystem, ClawStages.LOWER),
                        //new Wait(subsystem, 0.0),
                        new SetArmsStageCmd(subsystem, ArmsStages.TOP));
            } else {
                addCommands(
                        new SpinCmd(subsystem, 0, 0),
                        new ClawStageCmd(subsystem, ClawStages.UPPER),
                        new SetArmsStageCmd(subsystem, ArmsStages.SHRINK),
                        new SetRotationCmd(subsystem, 0.5),
                        new SlideUntilCmd(subsystem, position.get(), 1, true),
                        new ClawStageCmd(subsystem, ClawStages.LOWER),
                        //new Wait(subsystem, 0.0),
                        new SetArmsStageCmd(subsystem, ArmsStages.TOP));
                addRequirements(subsystem);
            }
        }

        @Override
        public void initialize() {
            super.initialize();
            IntakeManualGoToCmd.setEnabled(false);
        }

        @Override
        public void end(boolean interrupted) {
            IntakeManualGoToCmd.setEnabled(true);
        }
    }

    public static class SlideUntilCmd extends CommandBase {
        IntakeSubsystem subsystem;
        final int position;
        final int maxPosition = 3000;
        double power;
        boolean wait;

        public SlideUntilCmd(IntakeSubsystem subsystem, int position, double power, boolean wait) {
            this.power = power;
            this.subsystem = subsystem;
            this.wait = wait;
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
            subsystem.runWithoutEncoders();
            IntakeManualGoToCmd.setEnabled(false);
            if (subsystem.getMotorPosition() < position) {
                subsystem.setRawPower(power);
            }

        }

        @Override
        public boolean isFinished() {
            return (subsystem.getMotorPosition() > position) || !wait;
        }

        @Override
        public void end(boolean interrupted) {
            subsystem.setArmPower(0);
//            subsystem.setTargetPos(subsystem.getMotorPosition());
//            subsystem.armGoToTarget();
//            subsystem.runWithEncoders();
            IntakeManualGoToCmd.setEnabled(true);
        }
    }

    public static class RestartIntakeCmd extends SequentialCommandGroup {
        public RestartIntakeCmd(IntakeSubsystem subsystem) {
            addCommands(
                    new SpinCmd(subsystem, 0, 0),
                    new SetArmsStageCmd(subsystem, ArmsStages.TOP));
            addRequirements(subsystem);
        }

    }

    //public static class ManualIntakeCmd extends ParallelCommandGroup {
    //    public ManualIntakeCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> power, Supplier<Double> position) {
    //        addCommands
    //                (new SetZRotationSupplierCmd(intakeSubsystem, position),
    //                        new IntakeManualGoToCmd(intakeSubsystem, power));
    //        addRequirements(intakeSubsystem);
    //    }
    //}

    public static class SampleSubmIntakeCmd extends SequentialCommandGroup {
        public SampleSubmIntakeCmd(IntakeSubsystem intakeSubsystem) {
            final double
                    spinPower = 1,
                    middleTime = 0.5,
                    grabbingTime = 0.75,
                    holdingPower = 0.07;


            addCommands(
                    //new ParallelCommandGroup(
                    //        new SetArmsStageCmd(intakeSubsystem, ArmsStages.MIDDLE),
                    //        new SpinCmd(intakeSubsystem, -spinPower, middleTime)),
                    new ParallelCommandGroup(
                            new SetArmsStageCmd(intakeSubsystem, ArmsStages.BOTTOM),
                            new SpinCmd(intakeSubsystem, spinPower, grabbingTime)),
                    new SpinCmd(intakeSubsystem, holdingPower, -1));
            addRequirements(intakeSubsystem);
        }
    }

    public static class SampleGroundIntakeCmd extends SequentialCommandGroup {
        public SampleGroundIntakeCmd(IntakeSubsystem intakeSubsystem) {
            final double
                    spinPower = 1,
                    middleTime = 0.5,
                    grabbingTime = 0.3,
                    holdingPower = 0.07;


            addCommands(
                    //new ParallelCommandGroup(
                    //        new SetArmsStageCmd(intakeSubsystem, ArmsStages.MIDDLE),
                    //        new SpinCmd(intakeSubsystem, -spinPower, middleTime)),
                    new ParallelCommandGroup(
                            new SetArmsStageCmd(intakeSubsystem, ArmsStages.BOTTOM),
                            new SpinCmd(intakeSubsystem, spinPower, grabbingTime)),
                    new SpinCmd(intakeSubsystem, holdingPower, -1));
            addRequirements(intakeSubsystem);
        }
    }


    public static class SampleReverseIntakeCmd extends ParallelCommandGroup {
        public SampleReverseIntakeCmd(IntakeSubsystem intakeSubsystem) {
            final double spinPower = 1,
                    middleTime = 0.5,
                    grabbingTime = 0.75,
                    holdingPower = 0.05;

            addCommands(
                    new SetArmsStageCmd(intakeSubsystem, ArmsStages.MIDDLE),
                    new SpinCmd(intakeSubsystem, -spinPower, -1));

            addRequirements(intakeSubsystem);
        }
    }

    public static class ReturnArmForTransferCmd extends SequentialCommandGroup {
        public ReturnArmForTransferCmd(IntakeSubsystem intakeSubsystem, boolean initTime) {

            addCommands(
                    new SetArmsStageCmd(intakeSubsystem, ArmsStages.SHRINK),
                    new SetRotationCmd(intakeSubsystem, 0.5),
                    //new Wait(intakeSubsystem, 0.1),
                    //new SetArmsStageCmd(intakeSubsystem, ArmsStages.SHRINK),
                    new Wait(intakeSubsystem, 0.25),
                    new ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
                    new Wait(intakeSubsystem, 0.2),
                    new SlideHomeCmd(intakeSubsystem, initTime));
            addRequirements(intakeSubsystem);
        }

    }

    public static class ReturnArmForAutoTransferCmd extends SequentialCommandGroup {
        public ReturnArmForAutoTransferCmd(IntakeSubsystem intakeSubsystem, boolean initTime) {

            addCommands(
                    new SetArmsStageCmd(intakeSubsystem, ArmsStages.SHRINK),
                    new SetRotationCmd(intakeSubsystem, 0.5),
                    new ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
                    //new Wait(intakeSubsystem, 0),
                    new SlideHomeCmd(intakeSubsystem, initTime));
            addRequirements(intakeSubsystem);
        }

    }

    public static class ReturnArmForHMCmd extends SequentialCommandGroup {

        public ReturnArmForHMCmd(IntakeSubsystem intakeSubsystem) {

            addCommands(
                    new SetArmsStageCmd(intakeSubsystem, ArmsStages.SHRINK),
                    new SetRotationCmd(intakeSubsystem, 0.5),
                    //new Wait(intakeSubsystem, 0.0),
                    new ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
                    new Wait(intakeSubsystem, 1),
                    new SetArmsStageCmd(intakeSubsystem, ArmsStages.BOTTOM),
                    new ClawStageCmd(intakeSubsystem, ClawStages.UPPER),//for safety
                    new SlideUntilCmd(intakeSubsystem, 300, 1, true));
            addRequirements(intakeSubsystem);
        }

    }

    public static class Transfer extends SequentialCommandGroup {
        final int slidesBackAfterTransfer = 5;
        public static boolean transferring = false;
        IntakeSubsystem intakeSubsystem;

        public Transfer(IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addCommands(
                    new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                    new IntakeCommands.SpinCmd(intakeSubsystem, 0.06, -1),
                    new ParallelCommandGroup(
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem),
                            new ReturnArmForTransferCmd(intakeSubsystem, false)),
                    new SetPowerCmd(intakeSubsystem, -0.18),
                    //new Wait(intakeSubsystem, 0.1), //for safety
                    new DischargeGrabCmd(dischargeSubsystem),
                    new SpinCmd(intakeSubsystem, -0.13, -1),
                    new Wait(intakeSubsystem, 0.15),
                    new SetArmsStageCmd(intakeSubsystem, ArmsStages.TRANSFER),
                    new SetPowerCmd(intakeSubsystem, 0),
                    new SlideUntilCmd(intakeSubsystem, intakeSubsystem.minSlidesPos + slidesBackAfterTransfer, 0.4, false),
                    new SpinCmd(intakeSubsystem, 0, -1));
            addRequirements(intakeSubsystem, dischargeSubsystem); //may be unnecessary
        }


        @Override
        public void initialize() {
            super.initialize();
            transferring = true;
            DischargeCommands.MotorControl.setMode(DischargeCommands.MotorControl.Mode.DO_NOTHING);
        }

        @Override
        public void end(boolean interrupted) {
            transferring = false;
            intakeSubsystem.setArmPower(0);
        }
    }

    public static class AutoTransfer extends SequentialCommandGroup {
        final int slidesBackAfterTransfer = 10;
        public static boolean transferring = false;
        IntakeSubsystem intakeSubsystem;

        public AutoTransfer(IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addCommands(
                    new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                    new IntakeCommands.SpinCmd(intakeSubsystem, 0.06, -1),
                    new ParallelCommandGroup(
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem),
                            new ReturnArmForAutoTransferCmd(intakeSubsystem, false)),
                    new SetPowerCmd(intakeSubsystem, -0.18),
                    //new Wait(intakeSubsystem, 0.1), //for safety
                    new DischargeGrabCmd(dischargeSubsystem),
                    new SpinCmd(intakeSubsystem, -0.13, -1),
                    new Wait(intakeSubsystem, 0.15),
                    new SetArmsStageCmd(intakeSubsystem, ArmsStages.TRANSFER),
                    new SetPowerCmd(intakeSubsystem, 0),
                    new SlideUntilCmd(intakeSubsystem, intakeSubsystem.getMotorPosition() + slidesBackAfterTransfer, 0.6, false),
                    new SpinCmd(intakeSubsystem, 0, -1));
            addRequirements(intakeSubsystem, dischargeSubsystem); //may be unnecessary
        }


        @Override
        public void initialize() {
            super.initialize();
            transferring = true;
            DischargeCommands.MotorControl.setMode(DischargeCommands.MotorControl.Mode.DO_NOTHING);
        }

        @Override
        public void end(boolean interrupted) {
            transferring = false;
            intakeSubsystem.setArmPower(0);
        }
    }

    public static class WaitForTransferEnd extends CommandBase {
        public WaitForTransferEnd() {

        }

        @Override
        public void execute() {

        }

        @Override
        public boolean isFinished() {
            return !Transfer.transferring;
        }
    }

    public static class InverseTransfer extends SequentialCommandGroup {
        final int slidesBackAfterTransfer = 10;
        public static boolean transferring = false;

        public InverseTransfer(IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem) {
            addCommands(

                    new DischargeCommands.GoHomeCmd(dischargeSubsystem),
                    new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                    new SpinCmd(intakeSubsystem, 0.5, 0.1),
                    new SetArmsStageCmd(intakeSubsystem, ArmsStages.BOTTOM),
                    new SpinCmd(intakeSubsystem, 0.05, -1),
                    new SlideUntilCmd(intakeSubsystem, 1700, 1, false),
                    new ClawStageCmd(intakeSubsystem, ClawStages.LOWER),
                    new RestartIntakeCmd(intakeSubsystem));
            addRequirements(intakeSubsystem, dischargeSubsystem); //may be unnecessary
        }

        @Override
        public void initialize() {
            super.initialize();
            transferring = true;
        }

        @Override
        public void end(boolean interrupted) {
            transferring = false;
        }
    }

    public static class SetPositionCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        int pos;

        public SetPositionCmd(IntakeSubsystem intakeSubsystem, int pos) {
            this.pos = pos;
            this.intakeSubsystem = intakeSubsystem;
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPositionCorrection(1000);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class DontKillYourselfCmd extends SequentialCommandGroup {

        public DontKillYourselfCmd(IntakeSubsystem intakeSubsystem, int pos) {

            addCommands(new SetPositionCmd(intakeSubsystem, pos),
                    new StartIntakeCmd(intakeSubsystem));

        }
    }

    public static class IntakeShakeCmd extends CommandBase {
        double power, interval;
        ElapsedTime runtime = new ElapsedTime();
        IntakeSubsystem intakeSubsystem;
        int switched = 1;

        public IntakeShakeCmd(IntakeSubsystem intakeSubsystem, double power, double interval) {
            this.interval = interval;
            this.power = power;
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void execute() {
            if (runtime.seconds() <= interval) {
                intakeSubsystem.setArmPower(power * switched);
            } else {
                runtime.reset();
                switched *= -1;
            }
        }
    }
}