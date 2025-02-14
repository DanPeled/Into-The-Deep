package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
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
        double kp = 0.01;
        Point currentPos;
        double boost;
        double sensitivity;
        double rotation = 0;
        MecanumDrive mecanumDrive;
        Telemetry telemetry;
        boolean noRotation = false;
        double minPower = 0.2;

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
            double length = Math.hypot(localVector[0], localVector[1]);


            if (Math.abs(length) < minPower) {
                length = Math.signum(length) * minPower;
            }
            double speed = length;
            speed = Range.clip(speed, -1, 1);

            localVector[0] = Math.sin(MovementAngle) * speed;
            localVector[1] = Math.cos(MovementAngle) * speed;
            rotation = Utils.calcDeltaAngle(wantedAngle + 180, mecanumDrive.getAdjustedHeading()) * kp;

            if (noRotation) {
                mecanumDrive.drive(localVector[0], localVector[1], 0, boost);

            } else {
                mecanumDrive.drive(localVector[0], localVector[1], rotation / boost * 0.5, boost);
            }

        }

        @Override
        public boolean isFinished() {
            return (((Math.hypot(currentPos.x - x, currentPos.y - y) < sensitivity) || (900 <= wantedDistance))
                    && ((Math.abs(wantedAngle + 180 - mecanumDrive.getAdjustedHeading()) < 10) || noRotation));
        }

        @Override
        public void end(boolean interrupted) {
            mecanumDrive.drive(0, 0, 0, 0);
        }
    }

    public static class ConstantVelocityGotoCmd extends CommandBase {
        double x, y, wantedAngle, wantedDistance;
        double kp = 0.025;
        Point currentPos;
        double boost = 0.5;
        double sensitivity;
        double rotation = 0;
        double minPower = 0.04;
        MecanumDrive mecanumDrive;
        Telemetry telemetry;
        boolean noRotation = false;
        final double speed;

        public ConstantVelocityGotoCmd(Telemetry telemetry, MecanumDrive mecanumDrive, double x, double y,
                       double wantedAngle, double sensitivity, double wantedDistance, double speed) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.speed = speed;
            this.sensitivity = sensitivity;
            this.mecanumDrive = mecanumDrive;
            this.telemetry = telemetry;
            this.wantedDistance = wantedDistance;

            addRequirements(mecanumDrive);
        }

        public ConstantVelocityGotoCmd(Telemetry telemetry, MecanumDrive mecanumDrive, double x, double y,
                       double wantedAngle, double sensitivity, double speed) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.speed = speed;
            this.sensitivity = sensitivity;
            this.mecanumDrive = mecanumDrive;
            this.telemetry = telemetry;
            this.wantedDistance = -1;

            addRequirements(mecanumDrive);
        }

        public ConstantVelocityGotoCmd(Telemetry telemetry, MecanumDrive mecanumDrive, double x, double y,
                       double wantedAngle, double sensitivity, double speed, boolean noRotation) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.speed = speed;
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
            double length = Range.clip(speed, -1, 1);
            length += Math.signum(length) * minPower;
            localVector[0] = Math.sin(MovementAngle) * length;
            localVector[1] = Math.cos(MovementAngle) * length;
            rotation = Utils.calcDeltaAngle(wantedAngle + 180, mecanumDrive.getAdjustedHeading()) * kp;


            if (noRotation) {
                mecanumDrive.drive(localVector[0], localVector[1], 0, boost);

            } else {
                mecanumDrive.drive(localVector[0], localVector[1], (rotation / boost) * 0.5, boost);
            }

        }

        @Override
        public boolean isFinished() {
            return (((Math.hypot(currentPos.x - x, currentPos.y - y) < sensitivity) || (900 <= wantedDistance))
                    && ((Math.abs(wantedAngle + 180 - mecanumDrive.getAdjustedHeading()) < 10) || noRotation));
        }

        @Override
        public void end(boolean interrupted) {
            mecanumDrive.drive(0, 0, 0, 0);
        }
    }

    public static class SplineGotoCmd extends CommandBase {
        double x, y, wantedAngle = 0;
        double error, lastError, lastTime;
        double proportional, Integral, derivative;
        double kp = 0.025, ki = 0.0005, kd = -0.006;
        Point currentPos;
        double boost;
        double sensitivity;
        double rotation = 0;
        final double minPower = 0.04;
        MecanumDrive mecanumDrive;
        Point[] points = {new Point(0, 0), new Point(0, 0), new Point(0, 0)};

        public SplineGotoCmd(MecanumDrive mecanumDrive, Point p0, Point p1, Point p2, double boost, double sensitivity) {
            this.mecanumDrive = mecanumDrive;
            this.boost = boost;
            this.sensitivity = sensitivity;
            points[0] = p0;
            points[1] = p1;
            points[2] = p2;
            addRequirements(mecanumDrive);
        }

        @Override
        public void execute() {
            double t = Range.clip(findClosestT(mecanumDrive.getPosition(), points[0], points[1], points[2]) + 0.15, 0, 1);
            Point wantedPosition = new Point(Math.pow(1 - t, 2) * points[0].x + 2 * t * (1 - t) *
                    points[1].x + Math.pow(t, 2) * points[2].x,
                    Math.pow(1 - t, 2) * points[0].y + 2 * t * (1 - t) *
                            points[1].y + Math.pow(t, 2) * points[2].y);
            currentPos = mecanumDrive.getPosition();
            double[] localVector = {wantedPosition.x - currentPos.x, wantedPosition.y - currentPos.y};
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
            mecanumDrive.drive(localVector[0], localVector[1], rotation / 4, boost);

        }

        @Override
        public boolean isFinished() {
            return ((Math.hypot(currentPos.x - x, currentPos.y - y) < sensitivity) &&
                    (Math.abs(wantedAngle + 180 - mecanumDrive.getAdjustedHeading()) < 3));
        }

        public double findClosestT(Point current, Point P0, Point P1, Point P2) {
            double lower = 0.0, upper = 1.0, tolerance = 0.01; // Precision of the search
            while (upper - lower > tolerance) {
                double mid = (lower + upper) / 2.0;
                double left = bezierDistance(mid - tolerance, current.x, current.y, P0, P1, P2);
                double right = bezierDistance(mid + tolerance, current.y, current.y, P0, P1, P2);
                if (left < right) {
                    upper = mid;
                } else {
                    lower = mid;
                }
            }
            return (lower + upper) / 2.0;
        }

        private double bezierDistance(double t, double a, double b, Point P0, Point P1, Point P2) {
            // Calculate Bézier curve point at t
            double x_t = Math.pow(1 - t, 2) * P0.x + 2 * t * (1 - t) * P1.x + Math.pow(t, 2) * P2.x;
            double y_t = Math.pow(1 - t, 2) * P0.y + 2 * t * (1 - t) * P1.y + Math.pow(t, 2) * P2.y;

            // Calculate Euclidean distance to the point (a, b)
            return Math.sqrt(Math.pow(x_t - a, 2) + Math.pow(y_t - b, 2));
        }

    }

    public static class SetRotationCmd extends CommandBase {
        double wantedHeading;
        double error = 0, lastError = 0, proportional, lastTime = 0, Integral, derivative;
        public static double kp = 0.01, ki = 0.0005, kd = -0.006;//0.025
        MecanumDrive mecanumDrive;

        public SetRotationCmd(MecanumDrive mecanumDrive, double wantedHeading) {
            this.mecanumDrive = mecanumDrive;
            this.wantedHeading = wantedHeading;
            addRequirements(mecanumDrive);
        }

        @Override
        public void execute() {
//            double currentTime = (double) System.currentTimeMillis() / 1000;
//            double deltaTime = currentTime - lastTime;
//            if (lastTime != 0) {
//                error = Utils.calcDeltaAngle(wantedHeading + 180, mecanumDrive.getAdjustedHeading());
//                proportional = Range.clip(error, -100, 100) * kp;
//                Integral += Range.clip(error, -30, 30) * deltaTime;
//                if (Math.signum(Integral) != Math.signum(error)) {
//                    Integral = 0;
//                }
//                derivative = (lastError - error) / deltaTime;
//                mecanumDrive.drive(0, 0,
//                        (proportional + Integral * ki + derivative * kd +
//                                Math.signum(proportional + Integral * ki + derivative * kd) * 0.04) / 2,
//                        0.5);
//            }
//            lastError = error;
//            lastTime = currentTime;
            mecanumDrive.drive(0, 0, Utils.calcDeltaAngle(wantedHeading + 180, mecanumDrive.getAdjustedHeading()) * kp, 0.5);
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            Integral = 0;
            derivative = 0;
        }
    }

    public static class MoveSideToSideCmd extends CommandBase {
        MecanumDrive mecanumDrive;
        ElapsedTime time = new ElapsedTime();

        public MoveSideToSideCmd(MecanumDrive mecanumDrive) {
            this.mecanumDrive = mecanumDrive;
        }

        @Override
        public void initialize() {
            time.reset();
        }

        @Override
        public void execute() {

        }

        @Override
        public void end(boolean interrupted) {
            mecanumDrive.extraX = 0;
            mecanumDrive.extraY = 0;
            mecanumDrive.extraR = 0;
        }
    }

    public static class MoverServoCmd extends CommandBase {
        MecanumDrive mecanumDrive;
        double pos;

        public MoverServoCmd(MecanumDrive mecanumDrive, double pos) {
            this.mecanumDrive = mecanumDrive;
            this.pos = pos;
        }

        @Override
        public void initialize() {
            mecanumDrive.setMoverServo(pos);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
