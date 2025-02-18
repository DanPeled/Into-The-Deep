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
    final double limelightH = 41.5, sampleH = 3.8, limelightAngle = 27.6;
    int distance;
    public final double middleOfScreen = 300, tickPerCM = 19.34, distanceFromArmStart = 25;
    public double alignedY = 0;

    public LimelightSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public double getXDistance() {
        updateResults();
        result = limelight.getLatestResult();
        return result.getPythonOutput()[0];
    }

    public double getAngle() {
        updateResults();
        return result.getPythonOutput()[4];
    }

    public void setPipeline(Pipelines pipeline) {
        this.pipeline = pipeline.PIPELINE;
        limelight.pipelineSwitch(this.pipeline);
    }

    public double getCurrentPipeline() {
        return pipeline;
    }

    public int getYDistance() {
        updateResults();
        distance = Math.min((int) (((limelightH - sampleH) * Math.tan(Math.toRadians(-result.getPythonOutput()[1] / 240 * 42 + limelightAngle)) + distanceFromArmStart + 24) * tickPerCM), 1700);
        alignedY = distance;
        return distance;
    }

    public double getRawY() {
        updateResults();
        return result.getPythonOutput()[1];
    }

    public void startLimelight() {
        limelight.start();
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public void updateResults() {
        result = limelight.getLatestResult();
    }
}
