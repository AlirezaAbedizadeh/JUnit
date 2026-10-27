package autopilot.core;

import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import static org.junit.jupiter.api.Assertions.assertEquals;

class AutoDriverRemainingTest {
    private static final double CRUISING_SPEED_KMH = 80.0;
    private static final double CLEAR_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;
    private static final double RELEASED_BRAKE_PRESSURE = 0.0;
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
