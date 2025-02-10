package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    MultipleTelemetry telemetry;
    int pipeline = 0;
    LLResult result;
    final double limelightH = 0, sampleH = 3.8, limelightAngle = 30.5;
    double distance;
    public final double middleOfScreen = 300;

    public LimelightSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(pipeline);

    }

    public double getXDistance() {
        result = limelight.getLatestResult();
        return result.getPythonOutput()[0];
    }

    public double getAngle() {
        return result.getPythonOutput()[4];
    }

    public void setPipeline(Pipelines pipeline) {
        this.pipeline = pipeline.PIPELINE;
        limelight.pipelineSwitch(this.pipeline);
    }

    public double getCurrentPipeline() {
        return pipeline;
    }

    public double getYDistance() {
        distance = (limelightH - sampleH) * Math.tan(Math.toRadians(result.getPythonOutput()[1] + limelightAngle));
        return distance;
    }


}
