package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import java.util.EnumMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.config.PedroConfig;

/**
 * Central place to define reusable Pedro Pathing primitives.
 */
public final class PedroPathFactory {
    public enum PathId {
        FORWARD_24,
        STRAFE_RIGHT_24,
        SQUARE_24
    }

    private PedroPathFactory() {}

    /** Convenience accessor that uses the shared Pedro config. */
    public static PathChain getPath(PathId id) {
        return getPath(id, PedroConfig.getInstance());
    }

    public static PathChain getPath(PathId id, PedroConfig config) {
        switch (id) {
            case FORWARD_24:
                return forward(config, 24.0);
            case STRAFE_RIGHT_24:
                return strafeRight(config, 24.0);
            case SQUARE_24:
                return square(config, 24.0);
            default:
                throw new IllegalArgumentException("Unhandled path id: " + id);
        }
    }

    /** Exposes all prebuilt paths in a map for easy iteration. */
    public static Map<PathId, PathChain> getAllPaths(PedroConfig config) {
        Map<PathId, PathChain> map = new EnumMap<>(PathId.class);
        for (PathId id : PathId.values()) {
            map.put(id, getPath(id, config));
        }
        return map;
    }

    public static PathChain forward(PedroConfig config, double inches) {
        Pose start = origin();
        Pose end = new Pose(start.getX() + inches, start.getY(), start.getHeading(), PedroCoordinates.INSTANCE);
        return new PathChain(constraints(config), line(start, end, constraints(config)));
    }

    public static PathChain strafeRight(PedroConfig config, double inches) {
        Pose start = origin();
        Pose end = new Pose(start.getX(), start.getY() - inches, start.getHeading(), PedroCoordinates.INSTANCE);
        return new PathChain(constraints(config), line(start, end, constraints(config)));
    }

    public static PathChain square(PedroConfig config, double sideLengthInches) {
        PathConstraints c = constraints(config);
        Pose p0 = origin();
        Pose p1 = new Pose(p0.getX() + sideLengthInches, p0.getY(), p0.getHeading(), PedroCoordinates.INSTANCE);
        Pose p2 = new Pose(p1.getX(), p1.getY() + sideLengthInches, p0.getHeading(), PedroCoordinates.INSTANCE);
        Pose p3 = new Pose(p0.getX(), p2.getY(), p0.getHeading(), PedroCoordinates.INSTANCE);
        Path path1 = line(p0, p1, c);
        Path path2 = line(p1, p2, c);
        Path path3 = line(p2, p3, c);
        Path path4 = line(p3, p0, c);
        return new PathChain(c, path1, path2, path3, path4);
    }

    private static Pose origin() {
        return new Pose(0, 0, 0, PedroCoordinates.INSTANCE);
    }

    private static PathConstraints constraints(PedroConfig config) {
        return new PathConstraints(
            config.getPedroDefaults().getMaxVelocityIps(),
            config.getPedroDefaults().getMaxAngularVelocityRps(),
            config.getPedroDefaults().getMaxAccelIps2(),
            config.getPedroDefaults().getMaxAngularAccelRps2()
        );
    }

    private static Path line(Pose start, Pose end, PathConstraints constraints) {
        Path path = new Path(new BezierLine(start, end), constraints);
        path.setConstantHeadingInterpolation(start.getHeading());
        return path;
    }
}
