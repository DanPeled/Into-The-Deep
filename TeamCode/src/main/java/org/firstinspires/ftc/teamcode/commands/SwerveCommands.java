package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.teleop.DriveSubsystem;

import java.util.function.Supplier;

public class SwerveCommands {
    public static class PowerCmd extends CommandBase {
        Supplier<Double> x;
        Supplier<Double> y;
        Supplier<Double> r;
        Supplier<Double> boost;
        DriveSubsystem swerveDrive;
        Telemetry telemetry;

        public PowerCmd(Telemetry telemetry, DriveSubsystem swerveDrive, Supplier<Double> x,
                        Supplier<Double> y, Supplier<Double> r, Supplier<Double> boost) {
            this.x = x;
            this.y = y;
            this.r = r;
            this.boost = boost;
            this.swerveDrive = swerveDrive;
            this.telemetry = telemetry;

            addRequirements(swerveDrive);
        }

        @Override
        public void execute() {
            telemetry.addData("X", x.get());
            telemetry.addData("Y", y.get());
            telemetry.addData("TURN", r.get());
            swerveDrive.drive(x.get(), y.get(), r.get(), boost.get()/2);
        }
    }
    public static class GotoCmd extends CommandBase {
        double x,y, wantedAngle;
        double[] currentPos;
        double boost;
        double sensitivity;
        double kp = 0.02;
        DriveSubsystem swerveDrive;
        Telemetry telemetry;

        public GotoCmd(Telemetry telemetry, DriveSubsystem swerveDrive, double x, double y, double wantedAngle, double sensitivity, double boost) {
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
            currentPos = swerveDrive.getPosition();
            double[] localVector = {x - currentPos[0],y - currentPos[1]};
            double MovementAngle = Math.atan2(localVector[0], localVector[1]);
            double length =  Range.clip(Math.hypot(localVector[0], localVector[1]),-1,1);
            localVector[0] = Math.sin(MovementAngle * length);
            localVector[1] = Math.cos(MovementAngle * length);
            double angleDiff = Utils.calcDeltaAngle(wantedAngle, swerveDrive.getHeading()) * kp;
            swerveDrive.drive(localVector[0], localVector[1], angleDiff,boost);
        }

        @Override
        public boolean isFinished() {
            if(Math.hypot(currentPos[0] - x, currentPos[1] - y) < sensitivity){
                return true;
            }
            return false;
        }
    }
}
