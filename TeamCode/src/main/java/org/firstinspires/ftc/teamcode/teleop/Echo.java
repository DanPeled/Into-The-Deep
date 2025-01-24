package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.SteeringServo;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;

import java.util.function.Supplier;

@TeleOp
public class Echo extends CommandOpMode {

    SwerveDrive swerveDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    //LimeLightSubsystem limeLightSubsystem;

    Supplier<Double> swerveX, swerveY, swerveR;
    double lowSwerveSpeed = 0.3;

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
    Button systemBack;

    SwerveModule[] modules = new SwerveModule[4];

    @Override
    public void initialize() {
        SwerveDrive.minAngleError = 100000;
        driverGamepad = new GamepadEx(gamepad1);
        systemGamepad = new GamepadEx(gamepad2);

        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true);
        modules[0] = swerveDrive.fl;
        modules[1] = swerveDrive.fr;
        modules[2] = swerveDrive.bl;
        modules[3] = swerveDrive.br;
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        //limeLightSubsystem = new LimeLightSubsystem(hardwareMap, multipleTelemetry);
        register(swerveDrive, dischargeSubsystem, intakeSubsystem);
        initButtons();
        robotState = RobotState.NONE;
        controllersState = null;

        //init commands
        schedule(new SequentialCommandGroup(
                new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem),
                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
                new IntakeCommands.ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
                //new IntakeCommands.Wait(intakeSubsystem, 1),
                new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true),
                new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.TRANSFER),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)));
        IntakeCommands.IntakeManualGoToCmd.setEnabled(true);


        while (opModeInInit()) {
            super.run();
        }

        swerveR = () -> driverGamepad.getRightX();
        swerveX = () -> driverGamepad.getLeftX();
        swerveY = () -> driverGamepad.getLeftY();

        //swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
        //        driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
        //        () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

    }


    @Override
    public void run() {
        if (controllersState != robotState) {
            CommandScheduler.getInstance().clearButtons();

//            IntakeCommands.IntakeManualGoToCmd.endCommand();
            CommandScheduler.getInstance().cancel(swerveDrive.getDefaultCommand());
            CommandScheduler.getInstance().cancel(dischargeSubsystem.getDefaultCommand());
            CommandScheduler.getInstance().cancel(intakeSubsystem.getDefaultCommand());
            CommandScheduler.getInstance().setDefaultCommand(swerveDrive, new SwerveCommands.NoOpCommand(swerveDrive));
            CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem, new IntakeCommands.NoOpCommand(intakeSubsystem));
            CommandScheduler.getInstance().setDefaultCommand(dischargeSubsystem, new DischargeCommands.NoOpCommand(dischargeSubsystem));
//            driverA.whenHeld(new SwerveCommands.SetRotationCmd(swerveDrive, 0))
//                    .and(new Trigger(() -> !driverStart.get()));

            //systemLeftStickButton.whenPressed(new DischargeCommands.GearBoxClimbCmd(dischargeSubsystem));
            //systemRightStickButton.whenPressed(new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem));


            controllersState = robotState;
            switch (robotState) {
                case NONE:

                    driverDPadDown.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> 0.0, () -> -0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadUp.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> 0.0, () -> 0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadLeft.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> -0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadRight.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> 0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));

                    telemetry.addData("x", swerveX);
                    telemetry.addData("y", swerveY);
                    telemetry.update();
                    swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                            swerveX, swerveY, swerveR,
                            () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

                    intakeSubsystem.setDefaultCommand(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem, systemGamepad::getLeftY));

                    dischargeSubsystem.setDefaultCommand(new DischargeCommands.DischargeManualGotoCmd(
                            systemGamepad::getRightY, dischargeSubsystem, telemetry));
                    driverA.whenHeld(new SwerveCommands.SetRotationCmd(swerveDrive, 0));

                    systemA.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.WaitForTransferEnd(),
                            new SetStateCommands.ChamberStateCmd(), //change to chamber state
                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry)));

//                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem
//                                    , dischargeSubsystem.highChamberHeight, multipleTelemetry))); //go to chamber

                    systemY.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.WaitForTransferEnd(),
                            new SetStateCommands.BasketStateCmd(), //change to chamber state
                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highBasketHeight, telemetry))); //go to high basket

                    systemB.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.StartIntakeCmd(intakeSubsystem),
                            new SetStateCommands.IntakeStateCmd())).and(new Trigger(() -> !driverStart.get()));

                    systemX.whenPressed(new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem));

                    systemLeftStickButton.whenPressed(new DischargeCommands.GearBoxClimbCmd(dischargeSubsystem));

                    systemRightStickButton.whenPressed(new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem));

                    systemBack.whenPressed(new IntakeCommands.InverseTransfer(intakeSubsystem, dischargeSubsystem));

                    systemLeftBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem)));
                    break;
                case INTAKE:

                    systemDPadLeft.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> -0.1, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));
                    systemDPadRight.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> 0.1, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));

                    swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                            systemGamepad::getRightX, () -> 0.0, () -> 0.0,
                            () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), false));

                    intakeSubsystem.setDefaultCommand(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem,
                            systemGamepad::getLeftY));

                    systemA.whenPressed(new IntakeCommands.SampleReverseIntakeCmd(intakeSubsystem)).and(new Trigger(() -> IntakeCommands.Transfer.transferring));
                    systemA.whenReleased(new IntakeCommands.SampleIntakeCmd(intakeSubsystem)).and(new Trigger(() -> IntakeCommands.Transfer.transferring));

                    systemB.whenPressed(new IntakeCommands.RestartIntakeCmd(intakeSubsystem));

                    systemY.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.ReturnArmForHMCmd(intakeSubsystem),
                            new SetStateCommands.NoneStateCmd()));

                    systemX.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem)));

                    systemDPadUp.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 0.5));
                    systemDPadRight.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 0));
                    systemDPadLeft.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 1));

                    systemLeftBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem)));

                    break;
                case BASKET:

                    driverDPadDown.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> 0.0, () -> -0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadUp.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> 0.0, () -> 0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadLeft.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> -0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadRight.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> 0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));

                    swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                            swerveX, swerveY, swerveR,
                            () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

                    dischargeSubsystem.setDefaultCommand(new DischargeCommands.DischargeManualGotoCmd(
                            systemGamepad::getRightY, dischargeSubsystem, telemetry));

                    systemRightBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem)));

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

                    driverDPadDown.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> 0.0, () -> -0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadUp.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> 0.0, () -> 0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadLeft.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> -0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadRight.whileHeld(new SwerveCommands.PowerCmd(telemetry, swerveDrive, () -> 0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));


                    swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                            swerveX, swerveY, swerveR,
                            () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

                    dischargeSubsystem.setDefaultCommand(new DischargeCommands.DischargeManualGotoCmd(
                            systemGamepad::getRightY, dischargeSubsystem, telemetry));


                    systemRightBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)));


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
//        multipleTelemetry.addData("y",swerveDrive.getPosition().y);
//        multipleTelemetry.addData("bl pos", swerveDrive.bl.getPosition());
//        multipleTelemetry.addData("br pos", swerveDrive.br.getPosition());
//        multipleTelemetry.addData("fl pos", swerveDrive.fl.getPosition());
//        multipleTelemetry.addData("fr pos", swerveDrive.fr.getPosition());
//        multipleTelemetry.update();
//        multipleTelemetry.addData("x",swerveDrive.getPosition().x);
        for (int m = 0; m < 4; m++) {
            multipleTelemetry.addData("Error " + m, (modules[m].servo.error));
            multipleTelemetry.addData("target " + m, (modules[m].servo.getTargetAngle()));
            multipleTelemetry.addData("angle " + m, (modules[m].servo.getCurrentAngle()));
            multipleTelemetry.addData("power " + m, (modules[m].servo.power));
            multipleTelemetry.addData("P " + m, (modules[m].servo.error * SteeringServo.kp));
            multipleTelemetry.addData("I " + m, (modules[m].servo.integral * SteeringServo.ki));
            multipleTelemetry.addData("D " + m, (modules[m].servo.derivative * SteeringServo.kd));
        }
//        telemetry.addData("distance", swerveDrive.getDistance());
//        multipleTelemetry.addData("top", 50);
//        multipleTelemetry.addData("bottom", -50);
        multipleTelemetry.addData("is enabled", IntakeCommands.IntakeManualGoToCmd.isEnabled());
        multipleTelemetry.addData("transfering", IntakeCommands.Transfer.transferring);
        multipleTelemetry.addData("state", robotState);
        multipleTelemetry.addData("distance", swerveDrive.getDistance());
        multipleTelemetry.addData("angle", swerveDrive.getHeading());
        telemetry.addData("upper motor", dischargeSubsystem.getPosition());
        telemetry.addData("lower motor", dischargeSubsystem.getPosition2());
        multipleTelemetry.update();

//        multipleTelemetry.addData("posX", gamepad1.left_stick_x);
//        multipleTelemetry.addData("posY", gamepad1.left_stick_y);
//        multipleTelemetry.update();
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
        telemetry.addData("discharge default command", dischargeSubsystem.getDefaultCommand().getName());
        telemetry.addData("discharge current command", dischargeSubsystem.getCurrentCommand().getName());
        telemetry.addData("intake default command", intakeSubsystem.getDefaultCommand().getName());
        telemetry.addData("intake current command", intakeSubsystem.getCurrentCommand().getName());
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
        systemBack = new GamepadButton(systemGamepad, GamepadKeys.Button.BACK);
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
