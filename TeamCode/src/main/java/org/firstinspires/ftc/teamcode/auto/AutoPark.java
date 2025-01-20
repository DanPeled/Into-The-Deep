package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.opencv.core.Point;

@Disabled
@Autonomous
public class AutoPark extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    SwerveDrive swerveDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true,new Point(2.33,0.2));
        register(swerveDrive,dischargeSubsystem);
        schedule(new SequentialCommandGroup(
                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
                new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem),
                new IntakeCommands.Wait(intakeSubsystem, 1),
                new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem),
                new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.TRANSFER)));


        while (opModeInInit()){
            CommandScheduler.getInstance().run();
        };

        schedule(new SwerveCommands.PowerCmd(telemetry,swerveDrive, () -> 0.3, () -> 0.0, () -> 0.0, () -> 0.15, true).withTimeout(4000));//
    }
}
