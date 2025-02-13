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
import org.firstinspires.ftc.teamcode.SavedVariables;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.commands.SetStateCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Pipelines;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;

import java.util.function.Supplier;

@TeleOp
public class Echo extends CommandOpMode {

    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimelightSubsystem limeLightSubsystem;

    Supplier<Double> mecanumX, mecanumY, mecanumR;

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
    Button driverStart;
    Button systemBack;
    Button systemLeftStick, systemRightStick;


    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        systemGamepad = new GamepadEx(gamepad2);

        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, this);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        limeLightSubsystem = new LimelightSubsystem(hardwareMap, multipleTelemetry);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem, limeLightSubsystem);
        initButtons();
        robotState = RobotState.NONE;
        controllersState = null;

        //init commands
//        schedule(new SequentialCommandGroup(
//                new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem),
//                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
//                new IntakeCommands.ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
//                //new IntakeCommands.Wait(intakeSubsystem, 1),
//                new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true),
//                new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.TRANSFER),
//                new DischargeCommands.GoHomeCmd(dischargeSubsystem)));
//        IntakeCommands.IntakeManualGoToCmd.setEnabled(true);

        schedule(new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));

        while (opModeInInit()) {
            super.run();
        }
        limeLightSubsystem.startLimelight();
        limeLightSubsystem.setPipeline(Pipelines.YELLOW);
        //schedule(new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true));
        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, systemGamepad::getRightY, false, telemetry));


        mecanumX = () -> driverGamepad.getLeftX();
        mecanumY = () -> driverGamepad.getLeftY();
        mecanumR = () -> driverGamepad.getRightX();

//        swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
//                driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
//                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));
//
    }


    @Override
    public void run() {
        if (controllersState != robotState) {
            CommandScheduler.getInstance().clearButtons();

//            IntakeCommands.IntakeManualGoToCmd.endCommand();
            CommandScheduler.getInstance().cancel(mecanumDrive.getDefaultCommand());
            CommandScheduler.getInstance().cancel(dischargeSubsystem.getDefaultCommand());
            CommandScheduler.getInstance().cancel(intakeSubsystem.getDefaultCommand());
            CommandScheduler.getInstance().setDefaultCommand(mecanumDrive, new MecanumCommands.NoOpCommand(mecanumDrive));
            CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem, new IntakeCommands.NoOpCommand(intakeSubsystem));
            CommandScheduler.getInstance().setDefaultCommand(dischargeSubsystem, new DischargeCommands.NoOpCommand(dischargeSubsystem));
//            driverA.whenHeld(new SwerveCommands.SetRotationCmd(swerveDrive, 0))
//                    .and(new Trigger(() -> !driverStart.get()));

//            systemLeftStickButton.whenPressed(new DischargeCommands.GearBoxClimbCmd(dischargeSubsystem));
//            systemRightStickButton.whenPressed(new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem));


            controllersState = robotState;
            switch (robotState) {
                case NONE:
                    systemDPadRight.whenPressed(() -> SavedVariables.y = 4);
                    systemLeftStick.whenPressed(new DischargeCommands.ResetDischarge(dischargeSubsystem));
//                    systemRightStick.whenPressed(new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem));
                    systemRightBumper.whenPressed(new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem));

                    driverDPadDown.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> -0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadUp.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> 0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadLeft.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> -0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadRight.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));

                    telemetry.addData("x", mecanumX);
                    telemetry.addData("y", mecanumY);
                    telemetry.update();
                    mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
                            mecanumX, mecanumY, mecanumR, ()
                            -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5 + 0.5, true));

                    intakeSubsystem.setDefaultCommand(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem, systemGamepad::getLeftY));


                    driverA.whenHeld(new MecanumCommands.SetRotationCmd(mecanumDrive, 0));

                    systemA.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.WaitForTransferEnd(),
                            new SetStateCommands.ChamberStateCmd(), //change to chamber state
                            new DischargeCommands.GoToTarget(dischargeSubsystem.highChamberHeight)));

//                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem
//                                    , dischargeSubsystem.highChamberHeight, multipleTelemetry))); //go to chamber

                    systemY.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.WaitForTransferEnd(),
                            new SetStateCommands.BasketStateCmd(), //change to chamber state
                            //new DischargeCommands.GoToTarget(dischargeSubsystem.highBasketHeight),
                            new DischargeCommands.GoToTarget(dischargeSubsystem.highBasketHeight))); //ToDo: make it not go up randomly

                    systemB.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.StartIntakeCmd(intakeSubsystem),
                            new SetStateCommands.IntakeStateCmd())).and(new Trigger(() -> !driverStart.get()));

                    systemX.whenPressed(new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem));

//                    systemLeftStick.whenPressed(new DischargeCommands.GearBoxClimbCmd(dischargeSubsystem));
//                    systemRightStick.whenPressed(new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem));

                    systemBack.whenPressed(new SequentialCommandGroup(new IntakeCommands.DontKillYourselfCmd(intakeSubsystem, 800),
                            new SetStateCommands.IntakeStateCmd()));

                    systemLeftBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem)));
                    systemDPadUp.whenPressed(new LimelightCommands.LimelightIntake(limeLightSubsystem, intakeSubsystem,
                            dischargeSubsystem, mecanumDrive));
                    systemDPadDown.whenPressed(new SequentialCommandGroup(
                            new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, (int) limeLightSubsystem.getYDistance()),
                            new SetStateCommands.IntakeStateCmd()));
                    systemDPadLeft.whenHeld(new LimelightCommands.AlignXCmd(limeLightSubsystem, mecanumDrive));
                    break;
                case INTAKE:

                    systemDPadLeft.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> -0.1, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));
                    systemDPadRight.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.1, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));

                    mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
                            systemGamepad::getRightX, driverGamepad::getLeftY, () -> 0.0,
                            () -> systemGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5 + 0.2, false));

                    intakeSubsystem.setDefaultCommand(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem,
                            systemGamepad::getLeftY));

                    systemA.whenPressed(new IntakeCommands.SampleReverseIntakeCmd(intakeSubsystem)).and(new Trigger(() -> IntakeCommands.Transfer.transferring));
                    systemA.whenReleased(new IntakeCommands.SampleIntakeCmd(intakeSubsystem)).and(new Trigger(() -> IntakeCommands.Transfer.transferring));

                    systemB.whenPressed(new IntakeCommands.RestartIntakeCmd(intakeSubsystem));

//                    systemY.whenPressed(new SequentialCommandGroup(
//                            new IntakeCommands.ReturnArmForHMCmd(intakeSubsystem),
//                            new SetStateCommands.NoneStateCmd()));

                    systemX.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem)
                    ), false);

//                    systemDPadUp.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 0.5));
//                    systemDPadRight.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 0));
//                    systemDPadLeft.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 1));

                    systemDPadDown.whenPressed(
                            new IntakeCommands.SetRotationCmd(intakeSubsystem, (limeLightSubsystem.getAngle() + 90) / 180)
                    );


                    systemLeftBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem)));

                    break;
                case BASKET:

                    driverDPadDown.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> -0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadUp.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> 0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadLeft.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> -0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadRight.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));

                    mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
                            mecanumX, mecanumY, mecanumR,
                            () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5 + 0.5, true));


                    systemRightBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem)));

                    systemLeftBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem)));


                    systemA.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.ChamberStateCmd(), //change to chamber state
                            new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
                            new DischargeCommands.GoToTarget(dischargeSubsystem.highChamberHeight))); //go to chamber

                    systemY.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.BasketStateCmd(), //change to basket state
                            new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
                            new DischargeCommands.GoToTarget(dischargeSubsystem.highBasketHeight))); //go to high basket

                    break;
                case CHAMBER:
                    systemRightStick.whenPressed(new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem));


                    driverDPadDown.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> -0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadUp.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> 0.2, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadLeft.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> -0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));
                    driverDPadRight.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.2, () -> 0.0, () -> 0.0,
                            () -> 0.3, true));


                    mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
                            mecanumX, mecanumY, mecanumR,
                            () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5 + 0.5, true));


                    systemRightBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)));


                    systemLeftBumper.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.NoneStateCmd(),
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem)));


                    systemA.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.ChamberStateCmd(), //change to chamber state
                            new DischargeCommands.GoToTarget(dischargeSubsystem.highChamberHeight))); //go to chamber

                    systemY.whenPressed(new SequentialCommandGroup(
                            new SetStateCommands.BasketStateCmd(), //change to basket state
                            new DischargeCommands.GoToTarget(dischargeSubsystem.highBasketHeight))); //go to high basket

                    break;
            }
        }
        if (driverX.get() && driverStart.get()) {
            mecanumDrive.resetHeading();
        }

        //if (systemA.get() && controllersState == RobotState.INTAKE)
        //    systemX.whenPressed(new SequentialCommandGroup(
        //            new SetStateCommands.NoneStateCmd(),
        //            new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem)));
        super.run();
        telemetries();

    }

    private void telemetries() {
        telemetry.addData("discharge default command", dischargeSubsystem.getDefaultCommand().getName());
        telemetry.addData("discharge current command", dischargeSubsystem.getCurrentCommand().getName());
        telemetry.addData("intake default command", intakeSubsystem.getDefaultCommand().getName());
        telemetry.addData("intake current command", intakeSubsystem.getCurrentCommand().getName());

        multipleTelemetry.addData("x limelight", limeLightSubsystem.getXDistance());
        multipleTelemetry.addData("y limelight", limeLightSubsystem.getYDistance());
        multipleTelemetry.addData("angle limelight", limeLightSubsystem.getAngle());
        multipleTelemetry.addData("pipeline", limeLightSubsystem.getCurrentPipeline());
        telemetry.addData("y", limeLightSubsystem.getRawY());
        multipleTelemetry.addData("ticka", intakeSubsystem.getAveragePosition());
        multipleTelemetry.addData("tick1", intakeSubsystem.getMotorPosition());
        multipleTelemetry.addData("tick2", intakeSubsystem.getMotor2Position());
        multipleTelemetry.addData("cm", limeLightSubsystem.getYDistance() / limeLightSubsystem.tickPerCM);
//        multipleTelemetry.addData("lift mode", DischargeCommands.MotorControl.getMode());
//        multipleTelemetry.addData("lift target", DischargeCommands.MotorControl.getTargetPosition());
//        multipleTelemetry.addData("lift error", DischargeCommands.MotorControl.getTargetPosition() - dischargeSubsystem.getLiftPosInCM());
//        multipleTelemetry.addData("lift stay still target", DischargeCommands.MotorControl.getStayStillTarget());
//        telemetry.addData("y saved",SavedVariables.y);
        multipleTelemetry.update();
        telemetry.update();
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
        systemLeftStick = new GamepadButton(systemGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON);
        systemRightStick = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON);
    }
}
