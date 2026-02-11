package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "00 DecodeStarterTeleOp (Red)", group = "00 DecodeStarter")
public class DecodeStarterTeleOpRed extends DecodeStarterTeleOp {
    @Override
    public void init() {
        setTeamRed(true);
        super.init();
    }
}
