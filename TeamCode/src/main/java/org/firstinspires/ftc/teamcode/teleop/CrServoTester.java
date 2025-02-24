package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class CrServoTester extends CommandOpMode {

    CRServo servo;
    ElapsedTime time = new ElapsedTime();
    GamepadEx systemGamepad;
    GamepadButton systemA;
    GamepadButton systemB;
    GamepadButton systemY;
    double duration = 0;

    @Override
    public void initialize() {
        systemGamepad = new GamepadEx(gamepad2);
        systemA = new GamepadButton(systemGamepad, GamepadKeys.Button.A);
        systemB = new GamepadButton(systemGamepad, GamepadKeys.Button.B);
        systemY = new GamepadButton(systemGamepad, GamepadKeys.Button.Y);
        servo = hardwareMap.crservo.get("spinServo");
    }

    @Override
    public void run() {
        super.run();
        systemA.whenPressed(new Runnable() {
            @Override
            public void run() {
                time.reset();
                servo.setPower(1);
            }
        });
        systemY.whenPressed(new Runnable() {
            @Override
            public void run() {
                time.reset();
                servo.setPower(-1);
            }
        });
        systemB.whenPressed(() -> {
            duration = time.seconds();
            servo.setPower(0);
        });
        telemetry.addData("duration", duration);
        telemetry.addData("time", time.seconds());
        telemetry.update();

    }
}
