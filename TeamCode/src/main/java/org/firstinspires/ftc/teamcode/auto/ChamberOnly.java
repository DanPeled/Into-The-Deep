package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
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

//                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));
        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);
        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
//        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
//        schedule(new  IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true));
        while (opModeInInit()) {
            super.run();
        }
//

        schedule(new SequentialCommandGroup(

                chamberDiscahrge(1.78, 0),

                new ParallelCommandGroup(

                        new SequentialCommandGroup(
                                goToTwoSpeeds(1.84, 0.82, 0, 0.05, 0.65, 0.65, 0),

                                goToTwoSpeeds(2.5, 0.82, -90, 0.06, 0.8, 0.6, 0.15),

                                goToTwoSpeeds(2.92, 1.4, -90, 0.05, 0.65, 0.5, 0.15), //behind sample
                                goToTwoSpeeds(2.94, 0.7, -90, 0.08, 0.8, 0.65, 0.2), //observation

                                goToTwoSpeeds(3, 1.22, -90, 0.06, 0.8, 0.34, 0.17) //go back fast
                        ),

//                        new SequentialCommandGroup(
//                                goToTwoSpeeds(1.96, 0.82, -15, 0.045, 0.7, 0.7, 0),
//                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                moverGoGo(0.3),
//                                                wait(450), moverGoGo(0.5)),
//                                        goToTwoSpeeds(2.5, 0.83, -97, 0.06, 0.8, 0.65, 0.2)),
//                                goToTwoSpeeds(2.6, 0.9, -120, 0.06, 0.8, 0.5, 0.15),
//
//
//                                //goToTwoSpeeds(2.6, 0.88, -118, 0.045, 0.65, 0.55, 0.2),
//
//                                //moverGoGo(0.42),
//
//                                //goToTwoSpeeds(2.94, 1.4, -90, 0.05, 0.5, 0.35, 0.12), //behind sample
//
//                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                wait(600), moverGoGo(0.35)),
//                                        goToTwoSpeeds(2.8, 0.42, -90, 0.08, 0.8, 0.7, 0.2)), //observation
//
//
//                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                moverGoGo(0.38),
//                                                wait(200), moverGoGo(0.15)),
//                                        goToTwoSpeeds(2.96, 1.22, -90, 0.06, 0.8, 0.34, 0.17))  //behind sample
//                        ),
                        new DischargeCommands.GoHomeCmd(dischargeSubsystem)

                ),


                new SequentialCommandGroup(//special movement for fast intake here:

                        moverGoGo(0.45),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        wait(500), moverGoGo(0.385),
                                        wait(500), moverGoGo(0.3)),
                                goTo(2.6, 0.22, -90, 0.04, 1), //intake position
                                new SequentialCommandGroup(
                                        wait(400),
                                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1700))),
                        goTo(2.7, 0.22, -90, 0.04, 1),
                        new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem),
                        moverGoGo(0.5)

                ),

                chamberDiscahrge(1.8, 2),
                hpIntake(),
                chamberDiscahrge(1.8, 1),
                hpIntake(),
                chamberDiscahrge(1.8, 1)
        ));
    }

    public CommandBase chamberDiscahrge(double dischargeX, int transferMode) {

        if (transferMode == 0) {
            return new SequentialCommandGroup(

                    new ParallelDeadlineGroup(
                            new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight),
                            goTo(dischargeX, 0.88, 0, 0.05, 1.5)),

                    dischargeAction(dischargeX)

            );
        }

        if (transferMode == 1) {
            return new SequentialCommandGroup(

                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new IntakeCommands.AutoTransfer(intakeSubsystem, dischargeSubsystem),
                                    new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)),
                            new SequentialCommandGroup(
                                    wait(700),
                                    goTo(dischargeX, 0.86, -10, 0.06, 1.3))),

                    new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight),
                    dischargeAction(dischargeX)

            );
        } else {
            return new SequentialCommandGroup(

                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new IntakeCommands.AutoTransfer(intakeSubsystem, dischargeSubsystem),
                                    new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)),
                            new SequentialCommandGroup(
                                    wait(500),
                                    goTo(dischargeX, 0.76, 0, 0.08, 1.3),
                                    moverGoGo(0),
                                    goTo(dischargeX, 0.86, 0, 0.05, 1.3))),


                    new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight),
                    dischargeAction(dischargeX)
            );
        }
    }

    public CommandBase dischargeAction(double dischargeX) {
        return new ParallelDeadlineGroup(
                new DischargeCommands.SequentialRaceWrapper( // Custom wrapper for treating the sequence as a deadline
                        //new MecanumCommands.chamberWait(mecanumDrive),
                        new WaitCommand(600),
                        new DischargeCommands.AutoChamberDischargeCmd(dischargeSubsystem, telemetry)
                ),
                goToConstVelocity(dischargeX, 1.5, 0, 0.03, 0.5).withTimeout(5000)
        );
    }

    public CommandBase hpIntake() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new DischargeCommands.GoHomeCmd(dischargeSubsystem),
                        goTo(2.5, 0.55, -45, 0.03, 1),
                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1650)),
                new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem)

        );
    }

    private CommandBase goToConstVelocity(double x, double y,
                                          double wantedAngle, double sensitivity, double speed) {
        return new MecanumCommands.ConstantVelocityGotoCmd(telemetry, mecanumDrive, x, y, wantedAngle, sensitivity, speed);
//        return new MecanumCommands.GotoCmd()

    }

    private CommandBase goToTwoSpeeds(double x, double y,
                                      double wantedAngle, double sensitivity, double speed1, double speed2, double swap) {
        return new MecanumCommands.TwoSpeedsGotoCmd(telemetry, mecanumDrive, x, y, wantedAngle, sensitivity, speed1, speed2, swap);

    }


    private CommandBase goTo(double x, double y, double angle, double sensitivity, double speed) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, x, y, angle, sensitivity, speed);
    }

    private CommandBase wait(int mills) {
        return new WaitCommand(mills);
    }

    private CommandBase moverGoGo(double moveTo) {
        return new MecanumCommands.MoverServoCmd(mecanumDrive, moveTo);
    }

    @Override
    public void run() {
        AutoUtils.savePosition(mecanumDrive);
        super.run();
        telemetry.addData("x", mecanumDrive.getPosition().x);
        telemetry.addData("y", mecanumDrive.getPosition().y);
        telemetry.addData("intake pos", intakeSubsystem.getMotorPosition());
        telemetry.addData("elevator pos", dischargeSubsystem.getPosition());
        telemetry.update();
    }
}
