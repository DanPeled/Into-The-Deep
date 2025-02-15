package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.opencv.core.Point;

@Autonomous
public class BasketOnly extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimelightSubsystem limelightSubsystem;

    @Override
    public void initialize() {
        limelightSubsystem = new LimelightSubsystem(hardwareMap, multipleTelemetry);

        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(0.8, 0.22), 180, this);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem, limelightSubsystem);
        mecanumDrive.setHeading(0);
//        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);
        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
        while (opModeInInit()) {
            super.run();

        }

        schedule(new SequentialCommandGroup(

                new DischargeCommands.GoToTarget(dischargeSubsystem.highBasketHeight, dischargeSubsystem),
                new WaitCommand(400),

                new ParallelCommandGroup(
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.32, 0.265, 225, 0.03, 0.7),
                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1700)),
                //new WaitCommand(100),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.6, 0.59, 180, 0.005, 0.75),
                                new WaitCommand(100),
                                new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem)),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem))),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                                new ParallelCommandGroup(
                                        new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight),
                                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1700))

                        ),
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.35, 0.45, 225, 0.03, 1)
                ),

                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.34, 0.285, 225, 0.03, 1),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.32, 0.59, 180, 0.005, 0.75),
                                new WaitCommand(100),
                                new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem)),
                        new SequentialCommandGroup(new WaitCommand(800), new DischargeCommands.GoHomeCmd(dischargeSubsystem))),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                                new ParallelCommandGroup(
                                        new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight
                                        ), new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1200))),
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.35, 0.45, 225, 0.03, 1)
                ),

                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.34, 0.285, 225, 0.03, 1),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                new ParallelCommandGroup(
                        new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)),
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.2, 0.8, 180, 0.03, 1),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
                        )
                ),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.4, 0.8, 180, 0.03, 1),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.23, 0.7, 180, 0.01, 1),
                new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                                new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight)
                        ),
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.2, 0.4, 180, 0.03, 1)
                ),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.2, 0.3, 180, 0.03, 1),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem)


//                new LimelightCommands.LimelightIntake(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive)
        ));


    }

    @Override
    public void run() {
        AutoUtils.savePosition(mecanumDrive);
        super.run();
        multipleTelemetry.addData("posm1", intakeSubsystem.getMotorPosition());
        multipleTelemetry.addData("posm2", intakeSubsystem.getMotor2Position());
    }
}
