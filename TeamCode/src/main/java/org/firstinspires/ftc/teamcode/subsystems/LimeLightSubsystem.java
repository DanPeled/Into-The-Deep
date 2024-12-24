package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimeLightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    MultipleTelemetry telemetry;
    int pipeline = 0;
    LLResult result;
    final double limelightH = 0, sampleH = 3.8, limelightAngle = 0;
    double distance;

    public LimeLightSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(pipeline);



    }

    public double getXDistance() {
        result = limelight.getLatestResult();
        return result.getTx();
    }

    public void setPipeline(Pipelines pipeline) {
        limelight.pipelineSwitch(pipeline.PIPELINE);
    }
    public double getYDistance(){
        distance = (limelightH - sampleH) * Math.tan(Math.toRadians(result.getTy() + limelightAngle));
        return distance;
    }


}
