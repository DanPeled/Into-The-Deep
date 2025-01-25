package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.opencv.core.Point;

//@Disabled
@Autonomous(group = "park")
public class AutoPark extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(2.4, 0.2), 0, this);
        register(mecanumDrive, dischargeSubsystem);
//        schedule(new SequentialCommandGroup(
//                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
//                new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem),
//                new IntakeCommands.Wait(intakeSubsystem, 1),
//                new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true),
//                new DischargeCommands.GoHomeCmd(dischargeSubsystem),
//                new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.TRANSFER)));


        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
        }
        ;

        schedule(new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3.4, 0.2, 0, 0.05, 0.5, true));//
    }
}
