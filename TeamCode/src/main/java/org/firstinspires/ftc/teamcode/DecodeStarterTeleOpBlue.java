package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "00 DecodeStarterTeleOp (Blue)", group = "00 DecodeStarter")
public class DecodeStarterTeleOpBlue extends DecodeStarterTeleOp {
    @Override
    public void init() {
        setTeamRed(false);
        super.init();
    }
}
