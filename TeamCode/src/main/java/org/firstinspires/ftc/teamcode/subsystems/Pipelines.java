package org.firstinspires.ftc.teamcode.subsystems;

public enum Pipelines {
    RED(0),
    YELLOW(1),
    BLUE(5);
    public final int PIPELINE;

    private Pipelines(int pipeline) {
        this.PIPELINE = pipeline;
    }
}
