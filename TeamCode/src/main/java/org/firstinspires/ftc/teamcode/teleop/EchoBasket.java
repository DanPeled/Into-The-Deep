package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class EchoBasket extends Echo {
    @Override
    public void initialize() {
        super.initialize();
        mecanumDrive.setHeading(0);
    }
}
