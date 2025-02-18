package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.LimelightCommands;

@Disabled
@TeleOp
public class EchoLimelight extends Echo {
    @Override
    public void noneBindings() {
        super.noneBindings();
        systemB.whenPressed(new LimelightCommands.LimelightIntake(limeLightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive));

    }
}
