package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.params.provider.Arguments.arguments;

class AutoDriverTest {
    private static final double CRUISING_SPEED_KMH = 80.0;
    private static final double CLEAR_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;
    private static final double RELEASED_BRAKE_PRESSURE = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;
    private static final String SENSOR_FAILURE_MESSAGE = "Sensor reading unavailable";
    private static final GearBox.Gear DRIVING_GEAR = GearBox.Gear.D;

    @Test
    void tick_clearRoadAtTargetSpeed_returnsNormalHoldState() {
        AutoDriver driver = new AutoDriver(nominalSpeedSensor(), nominalObstacleSensor(),
                () -> CENTERED_LANE_OFFSET_METERS, CRUISING_SPEED_KMH);

        AutoDriver.DriveResult actual = driver.tick(DRIVING_GEAR);

        assertEquals(normalResult(0.0), actual);
    }

    @ParameterizedTest(name = "lane offset {0} m gives steering {1} degrees")
    @CsvSource({"1.0, -45.0", "-1.0, 45.0"})
    void tick_laneDeviation_steersInOppositeDirection(double laneOffset, double expectedSteering) {
        LaneSensor laneSensor = () -> laneOffset;
        AutoDriver driver = new AutoDriver(nominalSpeedSensor(), nominalObstacleSensor(),
                laneSensor, CRUISING_SPEED_KMH);

        AutoDriver.DriveResult actual = driver.tick(DRIVING_GEAR);

        assertEquals(normalResult(expectedSteering), actual);
    }

    @ParameterizedTest(name = "failed {0} sensor enters safe mode")
    @MethodSource("failedSensorCases")
    void tick_sensorFailure_returnsCompleteSafeState(String failedSensor,
                                                    SpeedSensor speedSensor,
                                                    ObstacleSensor obstacleSensor) {
        AutoDriver driver = new AutoDriver(speedSensor, obstacleSensor,
                () -> CENTERED_LANE_OFFSET_METERS, CRUISING_SPEED_KMH);

        AutoDriver.DriveResult actual = driver.tick(DRIVING_GEAR);

        assertEquals(new AutoDriver.DriveResult(0.0, SAFE_MODE_BRAKE_PRESSURE,
                AdaptiveCruise.Mode.BRAKE, false, true), actual, failedSensor);
    }

    static Stream<Arguments> failedSensorCases() {
        SpeedSensor failedSpeed = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };
        ObstacleSensor failedObstacle = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };
        return Stream.of(
                arguments("speed", failedSpeed, nominalObstacleSensor()),
                arguments("obstacle", nominalSpeedSensor(), failedObstacle)
        );
    }

    private static SpeedSensor nominalSpeedSensor() {
        return () -> CRUISING_SPEED_KMH;
    }

    private static ObstacleSensor nominalObstacleSensor() {
        return () -> CLEAR_DISTANCE_METERS;
    }

    private static AutoDriver.DriveResult normalResult(double steeringAngle) {
        return new AutoDriver.DriveResult(steeringAngle, RELEASED_BRAKE_PRESSURE,
                AdaptiveCruise.Mode.HOLD, false, false);
    }
}
