package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor motor1;
    private final DcMotor motor2;
    private final Servo xServo, zServo, gripServo;
    private final CRServo spinServo;
    private final int maxArmLength = 857;
    MultipleTelemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        motor1 = hardwareMap.dcMotor.get("intakeMotor1");
        motor2 = hardwareMap.dcMotor.get("intakeMotor2");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        xServo = hardwareMap.servo.get("xAxisServo");
        zServo = hardwareMap.servo.get("zAxisServo");
        gripServo = hardwareMap.servo.get("gripServo");
        spinServo = hardwareMap.crservo.get("spinServo");
        resetEncoders();
        this.telemetry = telemetry;

    }    // make one button that extends the arm and lowers the claw while opening it

    public void setArmPower(double power) {
        if (motor1.getCurrentPosition() <= 0 || motor2.getCurrentPosition() <= 0) {
            motor1.setPower(Range.clip(power, 0, 1));
            motor2.setPower(Range.clip(power, 0, 1));
        } else if (motor1.getCurrentPosition() >= maxArmLength || motor2.getCurrentPosition() >= maxArmLength) {
            motor1.setPower(Range.clip(power, -1, 0));
            motor2.setPower(Range.clip(power, -1, 0));
        } else {
            motor1.setPower(power);
            motor2.setPower(power);
        }
    }

    public void armGoToPos(int pos) {
        motor1.setTargetPosition(pos);
        motor1.setPower(0.4);
        motor2.setTargetPosition(pos);
        motor2.setPower(0.4);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setZServoPosition(double position) {
        zServo.setPosition(position);
    }

    public double getZServoPosition() {
        return zServo.getPosition();
    }

    public void setXServoPosition(double position) {
        xServo.setPosition(position);
    }

    public void setGripStage(GripStages stage) {
        gripServo.setPosition(stage.POSITION);
    }

    public double getGripServoPosition() {
        return gripServo.getPosition();
    }

    public void setSpinPower(double power) {
        spinServo.setPower(power);
    }

    public int getMotorPosition() {
        return motor1.getCurrentPosition();
    }

    public double getXServoPosition() {
        return xServo.getPosition();
    }

    public void resetEncoders() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void powerMode() {
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // make second button that while pressing it it goes to half of height and pushes things
    // away and then lowers one more stage and picks up the sample
    // button 3: checks angle and returns to base position
}
