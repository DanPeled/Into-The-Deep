package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.opencv.core.Point;

import java.util.function.Supplier;

public class SwerveCommands {
    public static class NoOpCommand extends CommandBase {
        public NoOpCommand(SwerveDrive swerveDrive) {
            addRequirements(swerveDrive);
        }

        @Override
        public boolean isFinished() {
            return false; // Runs indefinitely
        }
    }

    public static class PowerCmd extends CommandBase {
        Supplier<Double> x;
        Supplier<Double> y;
        Supplier<Double> r;
        Supplier<Double> boost;
        final SwerveDrive swerveDrive;
        final Telemetry telemetry;
        final boolean isFieldOriented;
        final double rotationModifier = 0.75;

        public PowerCmd(Telemetry telemetry, SwerveDrive swerveDrive, Supplier<Double> x,
                        Supplier<Double> y, Supplier<Double> r, Supplier<Double> boost, boolean isFieldOriented) {
            this.isFieldOriented = isFieldOriented;
            this.x = x;
            this.y = y;
            this.r = () -> r.get() * rotationModifier;
            this.boost = boost;
            this.swerveDrive = swerveDrive;
            this.telemetry = telemetry;


            addRequirements(swerveDrive);
        }

        @Override
        public void initialize() {
            swerveDrive.setFieldOriented(isFieldOriented);
        }

        @Override
        public void execute() {
            telemetry.addData("X", x.get());
            telemetry.addData("Y", y.get());
            telemetry.addData("TURN", r.get());
            swerveDrive.drive(x.get(), y.get(), r.get(), boost.get() / 2);
        }

        @Override
        public void end(boolean interrupted) {
            swerveDrive.drive(0, 0, 0, 0);
        }
    }

    public static class GotoCmd extends CommandBase {
        double x, y, wantedAngle;
        Point currentPos;
        double boost;
        double sensitivity;
        double kp = 0.02;
        final double minPower = 0.04;
        SwerveDrive swerveDrive;
        Telemetry telemetry;

        public GotoCmd(Telemetry telemetry, SwerveDrive swerveDrive, double x, double y,
                       double wantedAngle, double sensitivity, double boost) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.boost = boost;
            this.sensitivity = sensitivity;
            this.swerveDrive = swerveDrive;
            this.telemetry = telemetry;
//            SwerveDrive.minAngleError = 10;

            addRequirements(swerveDrive);
        }

        @Override
        public void execute() {
            currentPos = swerveDrive.getAdjustedPosition();
            double[] localVector = {x - currentPos.x, y - currentPos.y};
            double MovementAngle = Math.atan2(localVector[0], localVector[1]);
            double length = Range.clip(Math.hypot(localVector[0], localVector[1]), -1, 1);
            length += Math.signum(length) * minPower;
            localVector[0] = Math.sin(MovementAngle) * length;
            localVector[1] = Math.cos(MovementAngle) * length;
            double angleDiff = Utils.calcDeltaAngle(wantedAngle, swerveDrive.getHeading() - 180) * kp;
            swerveDrive.drive(localVector[0], localVector[1], angleDiff, boost);
            telemetry.addData("pos difference", Math.hypot(currentPos.x - x, currentPos.y - y));
        }

        @Override
        public boolean isFinished() {
            return Math.hypot(currentPos.x - x, currentPos.y - y) < sensitivity;
        }

        @Override
        public void end(boolean interrupted) {
            swerveDrive.idle();
        }
    }

    public static class SplineGotoCmd extends CommandBase {
        SwerveDrive swerveDrive;
        double boost, sensitivity;
        Point[] points;

        public SplineGotoCmd(SwerveDrive swerveDrive, Point p0, Point p1, Point p2, double boost, double sensitivity) {
            this.swerveDrive = swerveDrive;
            this.boost = boost;
            this.sensitivity = sensitivity;
            points[0] = p0;
            points[1] = p1;
            points[2] = p2;

        }
    }

    public static class SetPosition extends CommandBase {
        Point pos;
        SwerveDrive swerveDrive;

        public SetPosition(SwerveDrive swerveDrive, Point pos) {
            this.pos = pos;
            this.swerveDrive = swerveDrive;
        }

        @Override
        public void initialize() {
            swerveDrive.setPosition(pos);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    @Config
    public static class SetRotationCmd extends CommandBase {
        double wantedHeading;
        double error, lastError = 0, proportional, lastTime = 0, Integral, derivative;
        public static double kp = 0.011, ki = 0.001, kd = 0.002;
        SwerveDrive swerveDrive;

        public SetRotationCmd(SwerveDrive swerveDrive, double wantedHeading) {
            this.swerveDrive = swerveDrive;
            this.wantedHeading = wantedHeading;
            addRequirements(swerveDrive);
        }

        @Override
        public void execute() {
            double currentTime = (double) System.currentTimeMillis() / 1000;
            double deltaTime = currentTime - lastTime;
            if (lastTime != 0) {
                error = Utils.calcDeltaAngle(wantedHeading + 180, swerveDrive.getAdjustedHeading(0));
                proportional = error * kp;
                Integral += error * ki * deltaTime;
                if (Math.signum(Integral) != Math.signum(error)) {
                    Integral = 0;
                }
                derivative = (lastError - error) * kd / deltaTime;
                swerveDrive.drive(0, 0, proportional + Integral + derivative, 0.5);
            }
            lastError = error;
            lastTime = currentTime;
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }
}
