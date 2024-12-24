package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class DischargeSubsystem extends SubsystemBase {
    private final DcMotorEx lowerMotor, upperMotor;
    private Servo gearBoxServo, clawServo;
    double servoDischargePos = 0, servoClimbPos = 1;
    double clawServoHoldPos = 0.42, clawServoReleasePos = 0.83;
    MultipleTelemetry telemetry;
    double gearBoxRatio = 1;
    public double timeUp = 0;
    private double liftPosInCM;
    private double lastMotorTicks;

    public final double maxLiftPos = 1900;
    public final double minLiftPos = 20;
    public final double minClimbLiftPos = 170;

    public DischargeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        lowerMotor = hardwareMap.get(DcMotorEx.class, "lower");
        upperMotor = hardwareMap.get(DcMotorEx.class, "upper");
        gearBoxServo = hardwareMap.servo.get("gearBoxServo");
        clawServo = hardwareMap.servo.get("clawServo");
        lowerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gearBoxServo.setPosition(servoDischargePos);
        resetEncoders();
        lowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;
    }

    //fix: use gearbox ticks, not 1600
    public void setPower(double power) {
        setRawPower(calcPowerValue(power));
    }

    public void setRawPower(double power) {
        lowerMotor.setPower(power);
        upperMotor.setPower(power);
    }

    public double calcPowerValue(double power) {
        double currentPosInCM = getLiftPosInCM();
        if (currentPosInCM <= (gearBoxRatio == 1 ? minLiftPos : minClimbLiftPos)) {
            return (Range.clip(power, 0, 1));
        } else if (currentPosInCM >= maxLiftPos) {
            return ((Range.clip(power, -1, 0)));
        } else {
            return (power);
        }
    }

    public void climbMode() {
        gearBoxServo.setPosition(servoClimbPos);
        gearBoxRatio = 9;
        getLiftPosInCM();
    }

    public void dischargeMode() {
        gearBoxServo.setPosition(servoDischargePos);
        gearBoxRatio = 1;
        getLiftPosInCM();
    }

    public double getGearBoxRatio() {
        return gearBoxRatio;
    }

    public void resetEncoders() {
        lowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftPosInCM = 0;
        lastMotorTicks = 0;
        runWithoutEncoders();
    }

    public void setPosition(int pos) {
        lowerMotor.setTargetPosition(pos);
        upperMotor.setTargetPosition(pos);
        lowerMotor.setPower(1);
        upperMotor.setPower(1);
        lowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getPosition() {
        return upperMotor.getCurrentPosition();
    }

    public int getPosition2() {
        return lowerMotor.getCurrentPosition();
    }

    public void holdSample() {
        clawServo.setPosition(clawServoHoldPos);
    }

    public void releaseSample() {
        clawServo.setPosition(clawServoReleasePos);
    }

    public void testClaw(double clawPos) {
        clawServo.setPosition(clawPos);
    }

    public double getClawServoPosition() {
        return clawServo.getPosition();
    }

    public void runWithoutEncoders() {
        lowerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getLiftPosInCM() {
        int ct = getPosition();
        liftPosInCM += (ct - lastMotorTicks) / gearBoxRatio;
        lastMotorTicks = ct;
        return liftPosInCM;
    }
}
