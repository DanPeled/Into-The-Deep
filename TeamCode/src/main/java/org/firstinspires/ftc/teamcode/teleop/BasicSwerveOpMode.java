

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

import java.util.Random;


@TeleOp(group = "swerve")
public class BasicSwerveOpMode extends CommandOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private SwerveDrive swerveDrive;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    GamepadEx driverGamepad;
    public static double kp = 0.0022;
    public static double ki = 0.00000052;
    public static double kd = 0.0148;
    private double kChange = 1.03;
    double targetAngle;
    long lastTargetChangeTime;
    final double TARGET_INTERVAL = 1250;
    Random random = new Random();

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true);
        register(swerveDrive);

//        CommandScheduler.getInstance().setDefaultCommand(swerveDrive,new SwerveCommands.PowerCmd(telemetry, swerveDrive, driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX, () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
    }

    @Override
    public void run() {

        long currentTime = System.currentTimeMillis();
        //setHeading
        if (currentTime - lastTargetChangeTime > TARGET_INTERVAL) {
            targetAngle += 60;//random.nextInt(320)-160;
            targetAngle %= 360; // Keep within 0-360
            lastTargetChangeTime = currentTime;
        }
        swerveDrive.fl.setHeading(targetAngle, false);

        if (gamepad1.dpad_up) {
                   if (gamepad1.x) kp *= kChange;
                   else if (gamepad1.a) ki *= kChange;
                   else if (gamepad1.b) kd *= kChange;
               } else if (gamepad1.dpad_down) {
            if (gamepad1.x) kp /= kChange;
            else if (gamepad1.a) ki /= kChange;
            else if (gamepad1.b) kd /= kChange;
        }


        super.run();
        telemetry.addData("e", "e");
        telemetry.update();

        telemetry.addData("kp", kp);
        telemetry.addData("ki", ki*10000);
        telemetry.addData("kd", kd);

        telemetry.addData("ly: ", driverGamepad.getLeftX());
        telemetry.addData("rx: ", driverGamepad.getLeftY());
        telemetry.addData("lx: ", driverGamepad.getRightX());
        telemetry.addData("heading", swerveDrive.getHeading());
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Error", swerveDrive.fl.servo.error);
        dashboard.sendTelemetryPacket(packet);
    }
    public static double getKp(){
        return kp;
    }

    public static double getKi(){
        return ki;
    }

    public static double getKd(){
        return kd;
    }

}





