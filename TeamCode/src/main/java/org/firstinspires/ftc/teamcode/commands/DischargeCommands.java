package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;

import java.util.function.Supplier;

public class DischargeCommands {

    public static class DischargePowerCmd extends CommandBase {
        Supplier<Double> power;
        DischargeSubsystem dischargeSubsystem;
        Telemetry telemetry;

        public DischargePowerCmd(Supplier<Double> power, DischargeSubsystem dischargeSubsystem,
                                 Telemetry telemetry) {
            this.telemetry = telemetry;
            this.power = power;
            this.dischargeSubsystem = dischargeSubsystem;
            dischargeSubsystem.resetEncoders();
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void execute() {
            dischargeSubsystem.setPower((power.get()));
            telemetry.addData("power", dischargeSubsystem.calcPowerValue(power.get()));
        }
    }

    public static class DischargeGotoCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        Telemetry telemetry;
        private final int pos, sensitivity;
        private final double kp = 0.002;
        ElapsedTime elapsedTime = new ElapsedTime();


        public DischargeGotoCmd(DischargeSubsystem dischargeSubsystem, int pos, int sensitivity,
                                Telemetry telemetry) {

            this.dischargeSubsystem = dischargeSubsystem;
            this.pos = pos;
            this.sensitivity = sensitivity;
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
            elapsedTime.reset();
            dischargeSubsystem.setPosition(pos);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(dischargeSubsystem.getPosition() - pos) <= sensitivity;
        }

        @Override
        public void end(boolean interrupted) {
            dischargeSubsystem.timeUp = elapsedTime.seconds();
        }
    }

    public static class GoHomeCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        int lastTick;
        double lastTime = 0;
        final double maxDuration = 2.5;
        ElapsedTime elapsedTime = new ElapsedTime();

        public GoHomeCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
        }

        @Override
        public void initialize() {
            lastTick = dischargeSubsystem.getPosition();
            elapsedTime.reset();
            dischargeSubsystem.setPower(0);
            dischargeSubsystem.runWithoutEncoders();
        }

        @Override
        public void execute() {
            //if (dischargeSubsystem.getPosition() > 300) {
                dischargeSubsystem.setRawPower(-0.8);
            //} else {
            //    dischargeSubsystem.setRawPower(-0.8);
            //}
        }

        @Override
        public boolean isFinished() {
            if (dischargeSubsystem.getGearBoxRatio() == 1) { // finish check for normal gear
                if (elapsedTime.seconds() > maxDuration) {
                    return true;
                }

                double deltaTime = elapsedTime.seconds() - lastTime;
                if (deltaTime > 0.1) {
                    int deltaTick = dischargeSubsystem.getPosition() - lastTick;
                    lastTick = dischargeSubsystem.getPosition();
                    lastTime = elapsedTime.seconds();
                    if (Math.abs(deltaTick) <= 2) {
                        return true;
                    }
                }
            }
            else { // finish check for normal gear
                if (dischargeSubsystem.getLiftPosInCM() < dischargeSubsystem.minClimbLiftPos){
                    return true;
                }
            }
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            dischargeSubsystem.setPower(0);

            if (dischargeSubsystem.getGearBoxRatio() == 1)
                dischargeSubsystem.resetEncoders();
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

    public static class GearBoxSwapCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public GearBoxSwapCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            if (dischargeSubsystem.getGearBoxRatio() == 1) {
                dischargeSubsystem.climbMode();
            } else {
                dischargeSubsystem.dischargeMode();
            }
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

}
