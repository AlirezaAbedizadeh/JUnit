package autopilot.core;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;

class AutoDriverTest {

    // Issue 2: Fixed Magic Numbers - Replaced hardcoded values with class constants
    private static final double DEFAULT_SPEED_KMH = 80.0;
    private static final double DEFAULT_OBSTACLE_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET = 0.0;
    private static final double TARGET_CRUISE_SPEED_KMH = 80.0;
    private static final double SAFE_MODE_EXPECTED_BRAKE_PRESSURE = 30.0;
    private static final double ZERO_STEERING_ANGLE = 0.0;
    private static final double ZERO_BRAKE_PRESSURE = 0.0;

    // Issue 2: Fixed Magic Strings - Extracted hardcoded exception messages
    private static final String SPEED_SENSOR_ERROR_MSG = "Speed sensor failed";
    private static final String OBSTACLE_SENSOR_ERROR_MSG = "Obstacle sensor failed";

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    // Issue 4: Prevented Code Duplication by initializing base sensors in setUp
    @BeforeEach
    void setUp() {
        speedSensor = () -> DEFAULT_SPEED_KMH;
        obstacleSensor = () -> DEFAULT_OBSTACLE_DISTANCE_METERS;
        laneSensor = () -> CENTERED_LANE_OFFSET;

        autoDriver = new AutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor,
                TARGET_CRUISE_SPEED_KMH
        );
    }

    // Issue 1: Fixed Unclear Naming - Renamed 'test1' to indicate normal driving state
    @Test
    void tick_underNormalConditions_drivesSafelyWithNoBraking() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertNotNull(result);
        assertFalse(result.safeMode());
        assertNotNull(result.cruiseMode());
        assertEquals(ZERO_BRAKE_PRESSURE, result.brakePressure());
    }

    // Issue 1 & 4: Fixed Unclear Naming and Code Duplication using ParameterizedTest
    // Combined 'testLanePositive' and 'testLaneNegative' into a single test
    @ParameterizedTest(name = "lane offset {0} m -> expected steering angle < 0: {1}")
    @MethodSource("provideLaneDeviations")
    void tick_withLaneDeviation_correctsSteeringAngle(double offsetMeters, boolean expectsNegativeSteering) {
        laneSensor = () -> offsetMeters;
        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_CRUISE_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        if (expectsNegativeSteering) {
            assertTrue(result.steeringAngleDegrees() < ZERO_STEERING_ANGLE);
        } else {
            assertTrue(result.steeringAngleDegrees() > ZERO_STEERING_ANGLE);
        }
    }

    static Stream<Arguments> provideLaneDeviations() {
        return Stream.of(
                Arguments.of(1.0, true),   // positive offset expects negative steering
                Arguments.of(-1.0, false)  // negative offset expects positive steering
        );
    }

    // Issue 1, 3 & 4: Fixed Unclear Naming, Over-assertion, and Code Duplication
    // Combined 'testSpeedSensorFailure' and 'testObstacleSensorFailure' into a single parameterized test.
    // Reduced multiple assertions into a grouped validation structure logic.
    @ParameterizedTest(name = "Sensor failure throwing: {0}")
    @MethodSource("provideSensorFailures")
    void tick_onSensorFailure_activatesSafeModeProperly(SensorType failedSensor, String errorMsg) {
        if (failedSensor == SensorType.SPEED) {
            speedSensor = () -> { throw new SensorFailureException(errorMsg); };
        } else {
            obstacleSensor = () -> { throw new SensorFailureException(errorMsg); };
        }

        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_CRUISE_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        // Grouped assertion logic to avoid over-asserting independently
        assertAll("Safe mode validations",
            () -> assertTrue(result.safeMode(), "Must activate safe mode"),
            () -> assertEquals(SAFE_MODE_EXPECTED_BRAKE_PRESSURE, result.brakePressure(), "Must apply safe brake pressure"),
            () -> assertEquals(ZERO_STEERING_ANGLE, result.steeringAngleDegrees(), "Must not steer in safe mode"),
            () -> assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode(), "Cruise mode must be BRAKE"),
            () -> assertFalse(result.warningActive(), "Warning should not be active for sensor failures")
        );
    }

    static Stream<Arguments> provideSensorFailures() {
        return Stream.of(
                Arguments.of(SensorType.SPEED, SPEED_SENSOR_ERROR_MSG),
                Arguments.of(SensorType.OBSTACLE, OBSTACLE_SENSOR_ERROR_MSG)
        );
    }

    enum SensorType {
        SPEED, OBSTACLE
    }
}