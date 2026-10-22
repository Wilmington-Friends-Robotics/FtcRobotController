package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeStarterTeleOp (Blue)", group = "zz DecodeStarter Legacy")
public class DecodeStarterTeleOpBlue extends DecodeStarterTeleOp {
    @Override
    public void init() {
        setTeamRed(false);
        super.init();
    }
}
