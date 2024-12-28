package org.firstinspires.ftc.teamcode.commands;

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
            this.r = () -> r.get()*rotationModifier;
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
    }

    public static class GotoCmd extends CommandBase {
        double x, y, wantedAngle;
        Point currentPos;
        double boost;
        double sensitivity;
        double kp = 0.02;
        SwerveDrive swerveDrive;
        Telemetry telemetry;

        public GotoCmd(Telemetry telemetry, SwerveDrive swerveDrive, double x, double y, double wantedAngle, double sensitivity, double boost) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.boost = boost;
            this.sensitivity = sensitivity;
            this.swerveDrive = swerveDrive;
            this.telemetry = telemetry;

            addRequirements(swerveDrive);
        }

        @Override
        public void execute() {
            currentPos = swerveDrive.getAdjustedPosition();
            double[] localVector = {x - currentPos.x, y - currentPos.y};
            double MovementAngle = Math.atan2(localVector[0], localVector[1]);
            double length = Range.clip(Math.hypot(localVector[0], localVector[1]), -1, 1);
            localVector[0] = Math.sin(MovementAngle) * length;
            localVector[1] = Math.cos(MovementAngle) * length;
            double angleDiff = Utils.calcDeltaAngle(wantedAngle, swerveDrive.getHeading()) * kp;
            swerveDrive.drive(localVector[0], localVector[1], angleDiff, boost);
        }

        @Override
        public boolean isFinished() {
            if (Math.hypot(currentPos.x - x, currentPos.y - y) < sensitivity) {
                return true;
            }
            return false;
        }
    }

    public static class SetPosition extends CommandBase{
        Point pos;
        SwerveDrive swerveDrive;
        public SetPosition(SwerveDrive swerveDrive, Point pos){
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
}
