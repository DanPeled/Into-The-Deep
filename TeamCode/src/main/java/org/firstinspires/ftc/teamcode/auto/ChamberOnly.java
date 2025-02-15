package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.opencv.core.Point;

@Autonomous(group = "chamber")
public class ChamberOnly extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    Point pos = new Point(0, 0);

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(1.8, 0.2), 0, this);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem);
        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry),
                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));
        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);
//        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
        while (opModeInInit()) {
            super.run();
        }
//

        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 0.955, 0, 0.05, 1),
                        new DischargeCommands.GoToTarget(dischargeSubsystem.highChamberHeight, dischargeSubsystem)),
                new WaitCommand(300),

                new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry),
                new WaitCommand(300),

//                new InstantCommand(() -> mecanumDrive.resetPos(new Point(mecanumDrive.getPosition().x, 0.935))),
                new MecanumCommands.ConstantVelocityGotoCmd(telemetry, mecanumDrive, 1.8, 0.8, 0, 0.05, 0.8),
                new WaitCommand(100),
//                new InstantCommand(() -> this.pos = new Point(mecanumDrive.getPosition().x,mecanumDrive.getPosition().y)),
                new MecanumCommands.SetRotationCmd(mecanumDrive, -90).withTimeout(500),
                new WaitCommand(150),
//                new InstantCommand(() -> mecanumDrive.resetPos(pos)),
//                new m

                goToConstVelocity(2.68, 1.5, -90, 0.08, 0.8), //go back slow

                goToConstVelocity(2.91, 1.5, -90, 0.06, 0.45), //behind sample
                goToConstVelocity(2.91, 0.5, -90, 0.08, 1), //observation

                goToConstVelocity(2.91, 1.5, -90, 0.1, 1), //go back fast
                goToConstVelocity(2.91, 1.5, -90, 0.05, 0.65), //go back slow

                goToConstVelocity(3.15, 1.5, -90, 0.05, 0.45),  //behind sample
                goToConstVelocity(3.15, 0.5, -90, 0.08, 1) //observation


        ));
//        schedule(new SequentialCommandGroup(
//                new ParallelCommandGroup(
//                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 1.02, 0, 0.05, 0.9, true),
//                        new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry)),
//                new WaitCommand(200),
//                new ParallelCommandGroup(
//                        new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)),
//                new WaitCommand(200),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 0.8, 0, 0.14, 1),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.7, 0.8, 0, 0.1, 0.8),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.7, 1.55, 0, 0.04, 0.8),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.93, 1.55, 0, 0.05, 0.9),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.93, 0.4, 0, 0.05, 0.9, true),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3, 0.73, 0, 0.01, 1),
//                new WaitCommand(1400),
//                new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1350),
//                new WaitCommand(300),
//                new IntakeCommands.SampleIntakeCmd(intakeSubsystem),
//                new WaitCommand(300),
//
//                new ParallelCommandGroup(
//                        new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
//                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.75, 0.6, 0, 0.06, 1)
//                ),
//
//                new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight + 50, telemetry),
//                new WaitCommand(900),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.65, 1.05, 0, 0.05, 0.9),
//                new ParallelCommandGroup(
//                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1200),
//                        new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.75, 0.655, 0, 0.02, 1).withTimeout(150),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3, 0.9, 0, 0.015, 0.7),
//                new IntakeCommands.SlideGotoCmd(intakeSubsystem, 1350),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3, 0.7, 0, 0.01, 0.7),
//                new WaitCommand(900),
//                new IntakeCommands.SampleIntakeCmd(intakeSubsystem),
//                new WaitCommand(200),
//                new ParallelCommandGroup(
//                        new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
//                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.9, 0.6, 0, 0.06, 1)
//                ),
//
//                new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight + 50, telemetry),
//                new WaitCommand(900),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.92, 1.05, 0, 0.05, 0.9),
//                new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)));
    }

    private CommandBase goToConstVelocity(double x, double y,
                                          double wantedAngle, double sensitivity, double speed) {
        return new MecanumCommands.ConstantVelocityGotoCmd(telemetry, mecanumDrive, x, y, wantedAngle, sensitivity, speed);
//        return new MecanumCommands.GotoCmd()

    }

    private CommandBase goTo(double x, double y, double angle, double sensitivity, double speed) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, x, y, angle, sensitivity, speed);
    }

    @Override
    public void run() {
        telemetry.addData("x", mecanumDrive.getPosition().x);
        telemetry.addData("y", mecanumDrive.getPosition().y);
        AutoUtils.savePosition(mecanumDrive);
        telemetry.update();
        super.run();
    }
}
