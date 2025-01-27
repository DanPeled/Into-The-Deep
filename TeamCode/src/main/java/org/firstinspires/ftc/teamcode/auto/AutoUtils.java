package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class AutoUtils {
    public static void initCommands(CommandOpMode commandOpMode, DischargeSubsystem dischargeSubsystem, IntakeSubsystem intakeSubsystem) {
        commandOpMode.schedule(new SequentialCommandGroup(
                new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem),
                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
                new IntakeCommands.ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
                //new IntakeCommands.Wait(intakeSubsystem, 1),
                new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true),
                new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.TRANSFER),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)));
    }

    public static CommandBase inwardsPark(MecanumDrive mecanumDrive, Telemetry telemetry) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3.4, 0.2, 0, 0.04, 0.5, true);
    }
    public static CommandBase outwardsPark(MecanumDrive mecanumDrive, Telemetry telemetry) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.8, 0.2, 0, 0.04, 0.5, true);
    }
    public static CommandBase dischargeGotoChamber(DischargeSubsystem dischargeSubsystem, Telemetry telemetry){
        return new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry);
    }
    public static CommandBase dischargeGotoBasket(DischargeSubsystem dischargeSubsystem, Telemetry telemetry){
        return new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highBasketHeight, telemetry);
    }
    public static CommandBase chamberGoto(MecanumDrive mecanumDrive, Telemetry telemetry){
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 1.02, 0, 0.06, 0.7, true);
    }
    public static CommandBase inFrontOfChamberGoto(MecanumDrive mecanumDrive, Telemetry telemetry, double y){
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, y, 0, 0.03, 0.8);
    }
    public static CommandBase chamberDischarge(DischargeSubsystem dischargeSubsystem, Telemetry telemetry){
        return new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry);
    }
    public static CommandBase nextToBasketGoto(MecanumDrive mecanumDrive, Telemetry telemetry){
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.2, 0.8, 180, 0.05, 0.5);
    }
    public static CommandBase basketDischargePositionGoto(MecanumDrive mecanumDrive, Telemetry telemetry){
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.2, 0.35, 180, 0.03, 0.5);
    }
    public static CommandBase basketDischarge(DischargeSubsystem dischargeSubsystem){
        return new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem);
    }
    public static CommandBase dischargeGoHome(DischargeSubsystem dischargeSubsystem){
        return new DischargeCommands.GoHomeCmd(dischargeSubsystem);
    }




}
