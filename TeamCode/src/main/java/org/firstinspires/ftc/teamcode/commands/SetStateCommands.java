package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.teleop.Echo;

public class SetStateCommands {
    public static class NoneStateCmd extends CommandBase {
        @Override
        public void initialize() {
            Echo.setRobotState(RobotState.NONE);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class BasketStateCmd extends CommandBase {
        @Override
        public void initialize() {
            Echo.setRobotState(RobotState.BASKET);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class ChamberStateCmd extends CommandBase {
        @Override
        public void initialize() {
            Echo.setRobotState(RobotState.CHAMBER);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class IntakeStateCmd extends CommandBase {
        @Override
        public void initialize() {
            Echo.setRobotState(RobotState.INTAKE);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }


}
