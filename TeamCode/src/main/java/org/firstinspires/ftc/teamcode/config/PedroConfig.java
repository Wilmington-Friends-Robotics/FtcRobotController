package org.firstinspires.ftc.teamcode.config;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.io.InputStream;

/**
 * Loads {@code pedro_config.json} once and exposes strongly typed accessors for
 * Pedro Pathing constants shared across autonomous and TeleOp.
 */
public final class PedroConfig {
    private static final String RESOURCE_PATH = "/pedro_config.json";
    private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper()
            .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

    private static PedroConfig instance;

    private final ConfigData data;

    private PedroConfig(ConfigData data) {
        this.data = data;
    }

    /** Returns the cached configuration, loading it from resources if necessary. */
    public static synchronized PedroConfig getInstance() {
        if (instance == null) {
            instance = new PedroConfig(loadConfig());
        }
        return instance;
    }

    static synchronized void resetForTesting() {
        instance = null;
    }

    private static ConfigData loadConfig() {
        try (InputStream stream = PedroConfig.class.getResourceAsStream(RESOURCE_PATH)) {
            if (stream == null) {
                throw new IllegalStateException("Unable to locate " + RESOURCE_PATH + " on the classpath.");
            }
            return OBJECT_MAPPER.readValue(stream, ConfigData.class);
        } catch (IOException e) {
            throw new IllegalStateException("Failed to parse " + RESOURCE_PATH, e);
        }
    }

    public Deadwheel getDeadwheel() {
        return data.deadwheel;
    }

    public Pod getForwardPod() {
        return data.forwardPod;
    }

    public Pod getLateralPod() {
        return data.lateralPod;
    }

    public Pinpoint getPinpoint() {
        return data.pinpoint;
    }

    public Drive getDrive() {
        return data.drive;
    }

    public PedroDefaults getPedroDefaults() {
        return data.pedroDefaults;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    private static class ConfigData {
        @JsonProperty("deadwheel")
        Deadwheel deadwheel;

        @JsonProperty("forward_pod")
        Pod forwardPod;

        @JsonProperty("lateral_pod")
        Pod lateralPod;

        @JsonProperty("pinpoint")
        Pinpoint pinpoint;

        @JsonProperty("drive")
        Drive drive;

        @JsonProperty("pedro_defaults")
        PedroDefaults pedroDefaults;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Deadwheel {
        @JsonProperty("diameter_mm")
        private double diameterMm;

        @JsonProperty("radius_in")
        private double radiusIn;

        @JsonProperty("circumference_in")
        private double circumferenceIn;

        @JsonProperty("encoder_cpr")
        private int encoderCpr;

        @JsonProperty("ticks_per_inch")
        private double ticksPerInch;

        @JsonProperty("ticks_per_mm")
        private double ticksPerMm;

        public double getDiameterMm() {
            return diameterMm;
        }

        public double getRadiusIn() {
            return radiusIn;
        }

        public double getCircumferenceIn() {
            return circumferenceIn;
        }

        public int getEncoderCpr() {
            return encoderCpr;
        }

        public double getTicksPerInch() {
            return ticksPerInch;
        }

        public double getTicksPerMm() {
            return ticksPerMm;
        }
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Pod {
        @JsonProperty("longitudinal_offset_in_forward_positive")
        private double longitudinalOffsetInForwardPositive;

        @JsonProperty("lateral_offset_in_left_positive")
        private double lateralOffsetInLeftPositive;

        @JsonProperty("delta_direction")
        private String deltaDirection;

        @JsonProperty("notes")
        private String notes;

        public double getLongitudinalOffsetInForwardPositive() {
            return longitudinalOffsetInForwardPositive;
        }

        public double getLateralOffsetInLeftPositive() {
            return lateralOffsetInLeftPositive;
        }

        public String getDeltaDirection() {
            return deltaDirection;
        }

        public String getNotes() {
            return notes;
        }
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Pinpoint {
        @JsonProperty("i2c_name")
        private String i2cName;

        @JsonProperty("uses_internal_imu_heading")
        private boolean usesInternalImuHeading;

        public String getI2cName() {
            return i2cName;
        }

        public boolean isUsingInternalImuHeading() {
            return usesInternalImuHeading;
        }
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Drive {
        @JsonProperty("track_width_in")
        private double trackWidthIn;

        @JsonProperty("wheelbase_in")
        private double wheelbaseIn;

        public double getTrackWidthIn() {
            return trackWidthIn;
        }

        public double getWheelbaseIn() {
            return wheelbaseIn;
        }
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class PedroDefaults {
        @JsonProperty("max_velocity_ips")
        private double maxVelocityIps;

        @JsonProperty("max_accel_ips2")
        private double maxAccelIps2;

        @JsonProperty("max_angular_velocity_rps")
        private double maxAngularVelocityRps;

        @JsonProperty("max_angular_accel_rps2")
        private double maxAngularAccelRps2;

        public double getMaxVelocityIps() {
            return maxVelocityIps;
        }

        public double getMaxAccelIps2() {
            return maxAccelIps2;
        }

        public double getMaxAngularVelocityRps() {
            return maxAngularVelocityRps;
        }

        public double getMaxAngularAccelRps2() {
            return maxAngularAccelRps2;
        }
    }
}
