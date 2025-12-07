package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.PinpointFieldLocalizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.PedroConfig;

/**
 * Builds a Pedro Follower wired to the Pinpoint-based localizer and your drivetrain configuration.
 */
public final class PedroFollowerFactory {
    private PedroFollowerFactory() {}

    public static Follower build(HardwareMap hardwareMap, PinpointFieldLocalizer localizer, PedroConfig config) {
        FollowerConstants followerConstants = new FollowerConstants();
        followerConstants.defaults();

        MecanumConstants mec = new MecanumConstants()
                .leftFrontMotorName("front_left")
                .leftRearMotorName("back_left")
                .rightFrontMotorName("front_right")
                .rightRearMotorName("back_right")
                .maxPower(1.0)
                .useBrakeModeInTeleOp(true);

        PathConstraints constraints = new PathConstraints(
                config.getPedroDefaults().getMaxVelocityIps(),
                config.getPedroDefaults().getMaxAngularVelocityRps(),
                config.getPedroDefaults().getMaxAccelIps2(),
                config.getPedroDefaults().getMaxAngularAccelRps2()
        );

        return new FollowerBuilder(followerConstants, hardwareMap)
                .setLocalizer(new PedroPinpointLocalizer(localizer))
                .mecanumDrivetrain(mec)
                .pathConstraints(constraints)
                .build();
    }
}
