package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.opencv.core.Point;

import java.util.function.Supplier;

public class MecanumCommands {
    public static class NoOpCommand extends CommandBase {
        public NoOpCommand(MecanumDrive mecanumDrive) {
            addRequirements(mecanumDrive);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public static class PowerCmd extends CommandBase {
        Supplier<Double> x;
        Supplier<Double> y;
        Supplier<Double> r;
        Supplier<Double> boost;
        final MecanumDrive mecanumDrive;
        final Telemetry telemetry;
        final boolean isFieldOriented;
        final double rotationModifier = 0.75;

        public PowerCmd(Telemetry telemetry, MecanumDrive mecanumDrive, Supplier<Double> x,
                        Supplier<Double> y, Supplier<Double> r, Supplier<Double> boost, boolean isFieldOriented) {
            this.isFieldOriented = isFieldOriented;
            this.x = x;
            this.y = y;
            this.r = () -> r.get() * rotationModifier;
            this.boost = boost;
            this.mecanumDrive = mecanumDrive;
            this.telemetry = telemetry;


            addRequirements(mecanumDrive);
        }

        @Override
        public void initialize() {
            mecanumDrive.setFieldOriented(isFieldOriented);
        }

        @Override
        public void execute() {
//            telemetry.addData("X", x.get());
//            telemetry.addData("Y", y.get());
//            telemetry.addData("TURN", r.get());
            mecanumDrive.drive(x.get(), y.get(), r.get(), boost.get() / 2);
        }

        @Override
        public void end(boolean interrupted) {
            mecanumDrive.drive(0, 0, 0, 0);
        }
    }

    public static class GotoCmd extends CommandBase {
        double x, y, wantedAngle, wantedDistance;
        double error, lastError, lastTime;
        double proportional, Integral, derivative;
        double kp = 0.025, ki = 0.0005, kd = -0.006;
        Point currentPos;
        double boost;
        double sensitivity;
        double rotation = 0;
        final double minPower = 0.04;
        MecanumDrive mecanumDrive;
        Telemetry telemetry;
        boolean noRotation = false;

        public GotoCmd(Telemetry telemetry, MecanumDrive mecanumDrive, double x, double y,
                       double wantedAngle, double sensitivity, double wantedDistance, double boost) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.boost = boost;
            this.sensitivity = sensitivity;
            this.mecanumDrive = mecanumDrive;
            this.telemetry = telemetry;
            this.wantedDistance = wantedDistance;

            addRequirements(mecanumDrive);
        }

        public GotoCmd(Telemetry telemetry, MecanumDrive mecanumDrive, double x, double y,
                       double wantedAngle, double sensitivity, double boost) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.boost = boost;
            this.sensitivity = sensitivity;
            this.mecanumDrive = mecanumDrive;
            this.telemetry = telemetry;
            this.wantedDistance = -1;

            addRequirements(mecanumDrive);
        }

        public GotoCmd(Telemetry telemetry, MecanumDrive mecanumDrive, double x, double y,
                       double wantedAngle, double sensitivity, double boost, boolean noRotation) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.boost = boost;
            this.sensitivity = sensitivity;
            this.mecanumDrive = mecanumDrive;
            this.telemetry = telemetry;
            this.wantedDistance = -1;
            this.noRotation = noRotation;

            addRequirements(mecanumDrive);
        }

        @Override
        public void execute() {
            currentPos = mecanumDrive.getPosition();
            double[] localVector = {x - currentPos.x, y - currentPos.y};
            double MovementAngle = Math.atan2(localVector[0], localVector[1]);
            double length = Range.clip(Math.hypot(localVector[0], localVector[1]), -1, 1);
            length += Math.signum(length) * minPower;
            localVector[0] = Math.sin(MovementAngle) * length;
            localVector[1] = Math.cos(MovementAngle) * length;
            double currentTime = (double) System.currentTimeMillis() / 1000;
            double deltaTime = currentTime - lastTime;
            if (lastTime != 0) {
                error = Utils.calcDeltaAngle(wantedAngle + 180, mecanumDrive.getAdjustedHeading());
                proportional = Range.clip(error, -100, 100) * kp;
                Integral += Range.clip(error, -30, 30) * deltaTime;
                if (Math.signum(Integral) != Math.signum(error)) {
                    Integral = 0;
                }
                derivative = (lastError - error) / deltaTime;
                rotation = proportional + Integral * ki + derivative * kd + Math.signum(proportional + Integral * ki + derivative * kd) * 0.04;
                rotation = rotation * 0.65 / (boost * 0.7 + 0.3);
            }
            lastError = error;
            lastTime = currentTime;
            if (noRotation) {
                mecanumDrive.drive(localVector[0], localVector[1], 0, boost);

            } else {
                mecanumDrive.drive(localVector[0], localVector[1], rotation / 2.5, boost);
            }

        }

        @Override
        public boolean isFinished() {
            return (((Math.hypot(currentPos.x - x, currentPos.y - y) < sensitivity) || (mecanumDrive.getDistance() <= wantedDistance))
                    && ((Math.abs(wantedAngle + 180 - mecanumDrive.getAdjustedHeading()) < 3) || noRotation));
        }

        @Override
        public void end(boolean interrupted) {
            mecanumDrive.drive(0, 0, 0, 0);
        }
    }
}
