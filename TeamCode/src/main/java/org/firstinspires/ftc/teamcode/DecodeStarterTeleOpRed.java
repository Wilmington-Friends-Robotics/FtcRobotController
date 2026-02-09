package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeStarterTeleOp (Red)", group = "TeleOp")
public class DecodeStarterTeleOpRed extends DecodeStarterTeleOp {
    @Override
    public void init() {
        setTeamRed(true);
        super.init();
    }
}
