package org.firstinspires.ftc.teamcode.subsystems;

public enum Pipelines {
    RED(1),
    YELLOW(3),
    BLUE(5);
    public final int PIPELINE;

    private Pipelines(int pipeline) {
        this.PIPELINE = pipeline;
    }
}
