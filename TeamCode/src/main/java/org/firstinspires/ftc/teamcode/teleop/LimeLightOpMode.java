package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.limelight.LLResult;
import org.firstinspires.ftc.teamcode.limelight.LLResultTypes;
import org.firstinspires.ftc.teamcode.limelight.Limelight3A;

@Config
@TeleOp(group = "limelight")
public class LimeLightOpMode extends LinearOpMode {
    public static int pipeLine = 5;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.start();
        waitForStart();
        while (opModeIsActive()) {
            limelight.pipelineSwitch(pipeLine);
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
//                    Pose3D botpose = result.getBotpose();
                    multipleTelemetry.addData("tx", result.getTx());
                    multipleTelemetry.addData("ty", result.getTy());


                    multipleTelemetry.update();
//                    multipleTelemetry.addData("ty", result.getTyNC());
//                    multipleTelemetry.addData("Object size", result.getTa());
//                    multipleTelemetry.addData("Botpose", botpose.toString());
//                    multipleTelemetry.addData("Current Index", result.getPipelineIndex());
                    dashboardTelemetry.update();
                    telemetry.update();

                }
            }

        }
    }
}