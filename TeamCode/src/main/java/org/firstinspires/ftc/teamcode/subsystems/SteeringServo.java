package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.BasicSwerveOpMode;
import org.firstinspires.ftc.teamcode.utils.Utils;

import lombok.Getter;

@Config
public class SteeringServo {
    public static double noiseGain = 0.035;
    public static double noiseCuttoff = 2;
    public static double w = -0.7;
    public static double p = 0.25;

    //    private final double kp = 0.00334;

//    private final double kp = 0.0022;
//    private final double ki = 0.00000052;
//    private final double kd = 0.0148;

    //private final double kp = 0.0068;
    //private final double ki = 0.00000072;
    //private final double kd = 0.068;

    public static double kp = 0.0035;
    public static double ki = 0.0;
    public static double kd = 0.0;



    double minPower = 0.04;
    private double lastError = 0.0;
    private double integral = 0.0;
    private long lastTime = 0;
    double derivative;
    public double error;


    private boolean idle = false;
    private double power = 0;
    private CRServo servo;
    private AnalogInput encoder;
    private double angleOffset;
    @Getter
    double currentAngle;
    @Getter
    double targetAngle;

    double min = 0.007, max = 3.277;


    public SteeringServo(CRServo servo, AnalogInput encoder, double headingOffset) {
        this.servo = servo;
        this.encoder = encoder;
        this.angleOffset = headingOffset;
    }

    public void setPower(double power) {
        this.power = -power;
        servo.setPower(-power);
    }

    public void setTargetAngle(double target) {
        targetAngle = target;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    double getEncoderVoltage() {
        return encoder.getVoltage();
    }

    public double getCurrentAngle() {
        double v = getEncoderVoltage();
        currentAngle = ((v - min) / (max - min)) * 360 - angleOffset;
        return currentAngle;
    }

    public double getRawAngle() {
        double v = getEncoderVoltage();
        currentAngle = ((v - min) / (max - min)) * 360;
        return currentAngle;
    }

    public void setAngleOffset(double angle) {
        angleOffset = angle;
    }

    public double getAngleOffset() {
        return angleOffset;
    }

    public void zeroHeading() {
        angleOffset = 0;
        angleOffset = getCurrentAngle();
    }

    public double calcDeltaAngle(double target, double current) {
        double delta = target - current;
        if (delta > 180) {
            delta = delta - 360;
        } else if (delta < -180) {
            delta = 360 + delta;
        }
        return delta;
    }

    public void update() {
        long currentTime = System.currentTimeMillis();

        double currentAngle = 0;
        double deltaTime = 0;
        if (lastTime != 0) {
            currentAngle = getCurrentAngle();
            error = calcDeltaAngle(getTargetAngle(), currentAngle);
            deltaTime = currentTime - lastTime;
            integral += (error * deltaTime);
            derivative = (error - lastError) / deltaTime;
            power = kp * error +
                    ki * integral+
                    kd * derivative;

            if (power >= 0)
                setPower(Range.clip(power, minPower, 1));
            else
                setPower(Range.clip(power, -1, -minPower));

            //power = BasicSwerveOpMode.getKp() * error +
            //        BasicSwerveOpMode.getKi() * integral +
            //        BasicSwerveOpMode.getKd() * derivative;
//
            //if (power >= 0)
            //    setPower(Range.clip(power, BasicSwerveOpMode.getMin(), 1));
            //else
            //    setPower(Range.clip(power, -1, -BasicSwerveOpMode.getMin()));
        }

        lastError = error;
        lastTime = currentTime;


        //telemetry.addData("deltaTime", deltaTime);
        //telemetry.addData("error", error);
        //telemetry.addData("getTargetAngle", getTargetAngle());
        //telemetry.addData("power", power);
        //telemetry.addData("ki * integral", ki * integral);

        //TelemetryPacket packet = new TelemetryPacket();


        //
        //if(Math.abs(error) > noiseCuttoff) {
        //    power += Math.signum(power) * Math.random() * noiseGain;
        //}
    }


}
