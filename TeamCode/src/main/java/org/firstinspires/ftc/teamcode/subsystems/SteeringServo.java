package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.utils.Utils;

import lombok.Getter;

@Config
public class SteeringServo {
    public static double noiseGain = 0.035;
    public static double noiseCuttoff = 2;
    public static double w = -0.7;
    public static double p = 0.25;

    private boolean idle = false;
    private double power = 0;
    private CRServo servo;
    private AnalogInput encoder;
    private double angleOffset;
    @Getter
    double currentAngle ;
    @Getter double targetAngle;

    double min=0.007, max=3.277 ;

    public SteeringServo(CRServo servo, AnalogInput encoder, double headingOffset) {
        this.servo = servo;
        this.encoder = encoder;
        this.angleOffset = headingOffset;
    }

    public void setPower(double power) {
        this.power = power;
        servo.setPower(power);
    }

    public void setTargetAngle(double target){
        targetAngle = target;
    }

    public double getTargetAngle(){
        return targetAngle;
    }
    double getEncoderVoltage(){
        return encoder.getVoltage();
    }

    public double getCurrentAngle(){
        double v = getEncoderVoltage();
        currentAngle = ((v-min)/(max-min))*360 - angleOffset;
        return currentAngle;
    }
    public double getRawAngle(){
        double v = getEncoderVoltage();
        currentAngle = ((v-min)/(max-min))*360;
        return currentAngle;
    }
    public void setAngleOffset(double angle){
        angleOffset = angle;
    }
    public double getAngleOffset(){
        return angleOffset;
    }
    public void zeroHeading() {
        angleOffset = 0;
        angleOffset = getCurrentAngle();
    }

    public double calcDeltaAngle(double target, double current) {
        double delta = target - current;
        if(delta > 180){
            delta = delta - 360;
        }else if(delta < -180){
            delta = 360+ delta;
        }
        return delta;
    }

    public void update() {
        double currentAngle = getCurrentAngle();

        double error = calcDeltaAngle(targetAngle, currentAngle);
        double ne = -error /90;  // negative normalized error (-1..1)
        //0.00334
//        power = (w >= 0) ? (Utils.signRoot(ne) * (w) + ne * (1 - w)) * p :
//                (ne * Math.abs(ne) *(Math.abs(w)) + ne *(1 + w)) * p ;

        power = p * ne /*+ 0.15 * Math.signum(ne)*/;
        //
        if(Math.abs(error) > noiseCuttoff) {
            power += Math.signum(power) * Math.random() * noiseGain;
        }

        setPower(power);
    }


}
