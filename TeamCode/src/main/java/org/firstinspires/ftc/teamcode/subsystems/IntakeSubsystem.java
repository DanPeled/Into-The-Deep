package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor rMotor;
    private final DcMotor lMotor;
    private final Servo hServo; // claw up & down
    private final Servo rServo; // claw rotation
    private final Servo armsServo; // claw arms angle
    private final CRServo spinServo; // claw up & down
    private final int maxArmLength = 3000;
    MultipleTelemetry telemetry;

    public final double slidesSpeed = 1;
    public final double slidesLowSpeed = 0.25;


    public IntakeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        rMotor = hardwareMap.dcMotor.get("leftIntakeMotor");
        lMotor = hardwareMap.dcMotor.get("rightIntakeMotor");
        lMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hServo = hardwareMap.servo.get("heightServo");
        rServo = hardwareMap.servo.get("rotationServo");
        armsServo = hardwareMap.servo.get("armsServo");
        spinServo = hardwareMap.crservo.get("spinServo");
        resetEncoders();
        this.telemetry = telemetry;

    }    // make one button that extends the arm and lowers the claw while opening it

    public void setArmPower(double power) {
        if (rMotor.getCurrentPosition() <= 0 || lMotor.getCurrentPosition() <= 0) {
            rMotor.setPower(Range.clip(power, 0, 1));
            lMotor.setPower(Range.clip(power, 0, 1));
        } else if (rMotor.getCurrentPosition() >= maxArmLength || lMotor.getCurrentPosition() >= maxArmLength) {
            rMotor.setPower(Range.clip(power, -1, 0));
            lMotor.setPower(Range.clip(power, -1, 0));
        } else {
            rMotor.setPower(power);
            lMotor.setPower(power);
        }
    }
    public void setRawPower(double power){
        rMotor.setPower(power);
        lMotor.setPower(power);
    }

    public void armGoToPos(int pos) {
        rMotor.setTargetPosition(pos);
        lMotor.setTargetPosition(pos);
        lMotor.setPower(slidesSpeed);
        rMotor.setPower(slidesSpeed);
        rMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double getAveragePosition(){
        return ((double) rMotor.getCurrentPosition() + (double) lMotor.getCurrentPosition())/2;
    }

    public void setRotationServoPosition(double position) {
        rServo.setPosition(position);
    }

    public double getZServoPosition() {
        return rServo.getPosition();
    }

    public void setHServoPosition(double position) {
        hServo.setPosition(position);
    }

    public void setArmsStage(double stage) {
        armsServo.setPosition(stage);
    }

    public double getGripServoPosition() {
        return armsServo.getPosition();
    }

    public void setSpinPower(double power) {
        spinServo.setPower(power);
    }

    public int getMotorPosition() {
        return rMotor.getCurrentPosition();
    }

    public double getXServoPosition() {
        return hServo.getPosition();
    }

    public void resetEncoders() {
        rMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runWithoutEncoders();
    }
    public void runWithoutEncoders() {
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void powerMode() {
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // make second button that while pressing it it goes to half of height and pushes things
    // away and then lowers one more stage and picks up the sample
    // button 3: checks angle and returns to base position
}
