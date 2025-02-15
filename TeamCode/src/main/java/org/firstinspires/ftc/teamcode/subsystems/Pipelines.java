package org.firstinspires.ftc.teamcode.subsystems;

public enum Pipelines {
    RED(1),
    YELLOW(9),
    BLUE(7);
    public final int PIPELINE;

    private Pipelines(int pipeline) {
        this.PIPELINE = pipeline;
    }
}
