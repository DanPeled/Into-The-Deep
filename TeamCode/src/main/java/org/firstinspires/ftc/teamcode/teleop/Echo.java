package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.SetStateCommands;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

@TeleOp
public class Echo extends CommandOpMode {

    SwerveDrive swerveDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    //LimeLightSubsystem limeLightSubsystem;

    static RobotState robotState;
    static RobotState controllersState;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

    GamepadEx driverGamepad;
    GamepadEx systemGamepad;
    Button systemA, driverA;
    Button systemB, driverB;
    Button systemY, driverY;
    Button systemX, driverX;
    Button systemDPadDown, driverDPadDown;
    Button systemDPadUp, driverDPadUp;
    Button systemDPadRight, driverDPadRight;
    Button systemDPadLeft, driverDPadLeft;
    Button systemRightBumper, driverRightBumper;
    Button systemLeftBumper, driverLeftBumper;
    Button systemLeftStickButton, systemRightStickButton;
    Button driverStart;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        systemGamepad = new GamepadEx(gamepad2);

        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        //limeLightSubsystem = new LimeLightSubsystem(hardwareMap, multipleTelemetry);
        register(swerveDrive, dischargeSubsystem, intakeSubsystem);
        initButtons();
        robotState = RobotState.NONE;
        controllersState = null;


//        schedule(new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true));
//        schedule(new DischargeCommands.GoHomeCmd(dischargeSubsystem));

//        while (opModeInInit()) {
//            super.run();
//        }

        swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

    }


    @Override
    public void run() {
        if (controllersState != robotState) {
            CommandScheduler.getInstance().clearButtons();

            IntakeCommands.IntakeManualGoToCmd.endCommand();
            CommandScheduler.getInstance().cancel(swerveDrive.getDefaultCommand());
            CommandScheduler.getInstance().cancel(dischargeSubsystem.getDefaultCommand());
            CommandScheduler.getInstance().cancel(intakeSubsystem.getDefaultCommand());
            CommandScheduler.getInstance().setDefaultCommand(swerveDrive, new SwerveCommands.NoOpCommand(swerveDrive));
            CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem, new IntakeCommands.NoOpCommand(intakeSubsystem));
            CommandScheduler.getInstance().setDefaultCommand(dischargeSubsystem, new DischargeCommands.NoOpCommand(dischargeSubsystem));

            //systemLeftStickButton.whenPressed(new DischargeCommands.GearBoxClimbCmd(dischargeSubsystem));
            //systemRightStickButton.whenPressed(new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem));


            controllersState = robotState;
            switch (robotState) {
                case NONE:
                    swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                            driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
                            () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

                    schedule(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem,
                            systemGamepad::getLeftY));

                    dischargeSubsystem.setDefaultCommand(new DischargeCommands.DischargeManualGotoCmd(
                            systemGamepad::getRightY, dischargeSubsystem, telemetry));

                    systemA.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.ChamberStateCmd(), //change to chamber state
                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem
                                    , dischargeSubsystem.highChamberHeight, multipleTelemetry))); //go to chamber

                    systemY.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.BasketStateCmd(), //change to basket state
                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem
                                    , dischargeSubsystem.highBasketHeight, multipleTelemetry))); //go to high basket

                    systemB.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.StartIntakeCmd(intakeSubsystem),
                            new SetStateCommands.IntakeStateCmd())).and(new Trigger(() -> !driverStart.get()));

                    systemX.whenPressed(new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem));

                    systemLeftStickButton.whenPressed(new DischargeCommands.GearBoxClimbCmd(dischargeSubsystem));

                    systemRightStickButton.whenPressed(new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem));

                    break;
                case INTAKE:
                    swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                            systemGamepad::getRightX, () -> 0.0, () -> 0.0,
                            () -> 0.2, false));

                    schedule(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem,
                            systemGamepad::getLeftY));

                    systemA.whenPressed(new IntakeCommands.SampleIntakeCmd(intakeSubsystem));

                    systemB.whenPressed(new IntakeCommands.reStartIntakeCmd(intakeSubsystem));

                    systemY.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.ReturnArmForHMCmd(intakeSubsystem),
                            new SetStateCommands.NoneStateCmd()));

                    systemX.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                            new SetStateCommands.NoneStateCmd()));

                    systemDPadUp.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 0.5));
                    systemDPadRight.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 0));
                    systemDPadLeft.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 1));

                    break;
                case BASKET:
                    swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                            driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
                            () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

                    dischargeSubsystem.setDefaultCommand(new DischargeCommands.DischargeManualGotoCmd(
                            systemGamepad::getRightY, dischargeSubsystem, telemetry));

                    systemRightBumper.whenPressed(new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem));
                    systemLeftBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem)));


                    systemA.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.ChamberStateCmd(), //change to chamber state
                            new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem
                                    , dischargeSubsystem.highChamberHeight, multipleTelemetry))); //go to chamber

                    systemY.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.BasketStateCmd(), //change to basket state
                            new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem
                                    , dischargeSubsystem.highBasketHeight, multipleTelemetry))); //go to high basket

                    break;
                case CHAMBER:
                    swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                            driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
                            () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

                    dischargeSubsystem.setDefaultCommand(new DischargeCommands.DischargeManualGotoCmd(
                            systemGamepad::getRightY, dischargeSubsystem, telemetry));

                    systemRightBumper.whenPressed(new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry));
                    systemLeftBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem)));


                    systemA.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.ChamberStateCmd(), //change to chamber state
                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem
                                    , dischargeSubsystem.highChamberHeight, multipleTelemetry))); //go to chamber

                    systemY.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.BasketStateCmd(), //change to basket state
                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem
                                    , dischargeSubsystem.highBasketHeight, multipleTelemetry))); //go to high basket

                    break;
            }
        }
        if (driverX.get() && driverStart.get()) {
            swerveDrive.resetHeading();
        }
        //if (systemA.get() && controllersState == RobotState.INTAKE)
        //    systemX.whenPressed(new SequentialCommandGroup(
        //            new SetStateCommands.NoneStateCmd(),
        //            new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem)));

        super.run();
        telemetries();
    }

    private void telemetries() {
          multipleTelemetry.addData("posX", swerveDrive.getAdjustedPosition().x);
          multipleTelemetry.addData("posY", swerveDrive.getAdjustedPosition().y);
          multipleTelemetry.update();
//        telemetry.update();
//        telemetry.addData("state", robotState);
//        telemetry.addData("discharge slides pos", dischargeSubsystem.getLiftPosInCM());
//        telemetry.addData("intake slides pos", intakeSubsystem.getMotorPosition());
//        telemetry.addData("intake manual slides", IntakeCommands.IntakeManualGoToCmd.isEnabled());
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Error fl", (swerveDrive.fl.servo.error));
//        packet.put("target fl", Math.abs(swerveDrive.fl.servo.getTargetAngle()));
//        packet.put("Error fr", (swerveDrive.fr.servo.error));
//        packet.put("target fr", Math.abs(swerveDrive.fr.servo.getTargetAngle()));
//        packet.put("Error bl", (swerveDrive.bl.servo.error));
//        packet.put("target bl", Math.abs(swerveDrive.bl.servo.getTargetAngle()));
//        packet.put("Error br", (swerveDrive.br.servo.error));
//        packet.put("target br", Math.abs(swerveDrive.br.servo.getTargetAngle()));
//
//        packet.put("Min Bound", -25);
//        packet.put("Max Bound", 90);
//        packet.put("Min Error", -10);
//        packet.put("Max Error", 8);
//        dashboard.sendTelemetryPacket(packet);
        //telemetry.addData("discharge default command", dischargeSubsystem.getDefaultCommand().getName());
        //telemetry.addData("discharge current command", dischargeSubsystem.getCurrentCommand().getName());
        //telemetry.addData("intake default command", intakeSubsystem.getDefaultCommand().getName());
        //telemetry.addData("intake current command", intakeSubsystem.getCurrentCommand().getName());
    }

    public static void setRobotState(RobotState state) {
        robotState = state;
    }

    private void initButtons() {
        systemA = new GamepadButton(systemGamepad, GamepadKeys.Button.A);
        systemB = new GamepadButton(systemGamepad, GamepadKeys.Button.B);
        systemY = new GamepadButton(systemGamepad, GamepadKeys.Button.Y);
        systemX = new GamepadButton(systemGamepad, GamepadKeys.Button.X);
        systemDPadDown = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_DOWN);
        systemDPadUp = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_UP);
        systemDPadRight = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_RIGHT);
        systemDPadLeft = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_LEFT);
        systemRightBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        systemLeftBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.LEFT_BUMPER);
        systemLeftStickButton = new GamepadButton(systemGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON);
        systemRightStickButton = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        driverA = new GamepadButton(driverGamepad, GamepadKeys.Button.A);
        driverB = new GamepadButton(driverGamepad, GamepadKeys.Button.B);
        driverY = new GamepadButton(driverGamepad, GamepadKeys.Button.Y);
        driverX = new GamepadButton(driverGamepad, GamepadKeys.Button.X);
        driverDPadDown = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN);
        driverDPadUp = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP);
        driverDPadRight = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT);
        driverDPadLeft = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT);
        driverRightBumper = new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        driverLeftBumper = new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
        driverStart = new GamepadButton(driverGamepad, GamepadKeys.Button.START);

    }
}
