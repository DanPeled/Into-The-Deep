package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class EchoBasket extends Echo {
    @Override
    public void initialize() {
        super.initialize();
        mecanumDrive.setHeading(0);
    }
}
