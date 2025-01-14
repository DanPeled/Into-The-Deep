package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;

import java.util.function.Supplier;

public class DischargeCommands {

    public static class NoOpCommand extends CommandBase {
        public NoOpCommand(DischargeSubsystem dischargeSubsystem) {
            addRequirements(dischargeSubsystem);
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

    public static class DischargeManualGotoCmd extends CommandBase {
        Supplier<Double> power;
        DischargeSubsystem dischargeSubsystem;
        Telemetry telemetry;
        ElapsedTime elapsedTime = new ElapsedTime();
        double lastTimeMilli = 0;


        public DischargeManualGotoCmd(Supplier<Double> power, DischargeSubsystem dischargeSubsystem,
                                      Telemetry telemetry) {
            this.telemetry = telemetry;
            this.power = power;
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize(){
            elapsedTime.reset();
        }

        @Override
        public void execute() {
            double timeMilli = elapsedTime.milliseconds();
            double deltaTime = timeMilli - lastTimeMilli;
            if (Math.abs(power.get()) > 0.25) {
                dischargeSubsystem.runToPosition();
                dischargeSubsystem.changeTargetPos(-power.get() * deltaTime * (dischargeSubsystem.manualTicksPerSecond/1000.0));
                dischargeSubsystem.goToTarget();
            }
            lastTimeMilli = timeMilli;
        }
    }

    public static class DischargeGotoCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        Telemetry telemetry;
        private final int pos;


        public DischargeGotoCmd(DischargeSubsystem dischargeSubsystem, int pos, Telemetry telemetry) {

            this.dischargeSubsystem = dischargeSubsystem;
            this.pos = pos;
            this.telemetry = telemetry;
            addRequirements(dischargeSubsystem);

        }

//        @Override
//        public void execute() {
//            dischargeSubsystem.setPower(kp * (pos - dischargeSubsystem.getPosition2()));
//            telemetry.addData("power", kp * (pos - dischargeSubsystem.getPosition2()));
//            telemetry.update();
//        }


        @Override
        public void initialize() {
            dischargeSubsystem.runToPosition();
            dischargeSubsystem.setTargetPosInTicks(pos);
            dischargeSubsystem.goToTarget();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class GoHomeCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        int lastTick;
        //        double lastTime = 0;
        final double maxDuration;
        final int minTargetOffset = 50;
        ElapsedTime elapsedTime = new ElapsedTime();

        public GoHomeCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            maxDuration = 3;
        }

        public GoHomeCmd(DischargeSubsystem dischargeSubsystem, double maxDuration) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.maxDuration = maxDuration;
        }

        @Override
        public void initialize() {
            lastTick = dischargeSubsystem.getPosition();
            elapsedTime.reset();
            dischargeSubsystem.setPower(0);
            dischargeSubsystem.runWithoutEncoders();
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void execute() {
            if (dischargeSubsystem.getPosition() > 300)
                dischargeSubsystem.setRawPower(-dischargeSubsystem.slidesSpeed);
            else
                dischargeSubsystem.setRawPower(-dischargeSubsystem.slidesLowSpeed);

        }

        @Override
        public boolean isFinished() {
            return dischargeSubsystem.isHome() || elapsedTime.seconds() > maxDuration;
//            if (dischargeSubsystem.getGearBoxRatio() == 1) { // finish check for normal gear
//                if (elapsedTime.seconds() > maxDuration) {
//                    return true;
//                }
//                if (elapsedTime.seconds() < 0.4)
//                    return false;
//
//                double deltaTime = elapsedTime.seconds() - lastTime;
//                if (deltaTime > 0.2) {
//                    int deltaTick = dischargeSubsystem.getPosition() - lastTick;
//                    lastTick = dischargeSubsystem.getPosition();
//                    lastTime = elapsedTime.seconds();
//                    if (Math.abs(deltaTick) <= 5) {
//                        return true;
//                    }
//
//                }
//            }
//            else { // finish check for normal gear
//                if (dischargeSubsystem.getLiftPosInCM() < dischargeSubsystem.minClimbLiftPos){
//                    return true;
//                }
//            }
//            return false;
        }

        @Override
        public void end(boolean interrupted) {
            dischargeSubsystem.setPower(0);

            if (dischargeSubsystem.getGearBoxRatio() == 1 && !interrupted) {
                dischargeSubsystem.minLiftPos = dischargeSubsystem.getPosition() + minTargetOffset;
                dischargeSubsystem.setTargetPosInTicks(dischargeSubsystem.getPosition() + minTargetOffset);
                dischargeSubsystem.resetEncoders();
            }
        }
    }

    public static class DischargeGrabCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public DischargeGrabCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
//            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            dischargeSubsystem.holdSample();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class DischargeReleaseCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public DischargeReleaseCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            dischargeSubsystem.releaseSample();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class DischargeClawTestCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        Telemetry telemetry;
        Supplier<Double> ClawPos;

        public DischargeClawTestCmd(Supplier<Double> ClawPos ,DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.telemetry = telemetry;
            this.ClawPos = ClawPos;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void execute() {
            double c = (ClawPos.get() + 1) / 2;
            dischargeSubsystem.testClaw(c);
            telemetry.addData("claw pos (joy stick)", c);
            telemetry.addData("claw pos (servo)", dischargeSubsystem.getClawServoPosition());
        }
    }

    public static class GearBoxClimbCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public GearBoxClimbCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            dischargeSubsystem.climbMode();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class GearBoxDischargeCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public GearBoxDischargeCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            dischargeSubsystem.dischargeMode();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
    public static class ChamberDischargeCmd extends SequentialCommandGroup{
        public ChamberDischargeCmd(DischargeSubsystem dischargeSubsystem, Telemetry telemetry){
            addCommands(
                    new DischargeGotoCmd(dischargeSubsystem,dischargeSubsystem.highChamberHeight-250,telemetry),
                    new WaitCommand(300),
                    new DischargeReleaseCmd(dischargeSubsystem), new WaitCommand(100),
                    new DischargeCommands.GoHomeCmd(dischargeSubsystem));
            addRequirements(dischargeSubsystem);
        }
    }

    //public static class ChamberDischarge extends SequentialCommandGroup {
    //    public SampleIntakeCmd(IntakeSubsystem intakeSubsystem) {
    //        final double
    //                spinPower = 1,
    //                middleTime = 0.75,
    //                grabbingTime = 0.5,
    //                holdingPower = 0.05;
//
//
    //        addCommands(
    //                new ParallelCommandGroup(
    //                        new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.MIDDLE),
    //                        new IntakeCommands.SpinCmd(intakeSubsystem, -spinPower, middleTime)),
    //                new ParallelCommandGroup(
    //                        new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.BOTTOM),
    //                        new IntakeCommands.SpinCmd(intakeSubsystem, spinPower, grabbingTime)),
    //                new IntakeCommands.SpinCmd(intakeSubsystem, holdingPower, -1));
    //    }
    //}

}
