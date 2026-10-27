package autopilot.core;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;

class BrakeControllerTest {

    private BrakeController brakeController;
    private static final double TOLERANCE = 1e-5;

    @BeforeEach
    void setUp() {
        brakeController = new BrakeController();
    }

    @ParameterizedTest
    @EnumSource(value = GearBox.Gear.class, names = "P")
    void testBrakingInParkThrowsException(GearBox.Gear gear) {
        assertThrows(IllegalStateException.class, () -> brakeController.apply(50.0, 50.0, gear));
        assertThrows(IllegalStateException.class, () -> brakeController.emergencyBrake(gear));
    }

    @ParameterizedTest
    @EnumSource(value = GearBox.Gear.class, mode = EnumSource.Mode.EXCLUDE, names = "P")
    void testBrakingInValidGears(GearBox.Gear gear) {
        assertDoesNotThrow(() -> brakeController.apply(50.0, 50.0, gear));
    }

    @Test
    void testNegativeInputsThrowException() {
        assertThrows(IllegalArgumentException.class, () -> brakeController.apply(-10.0, 50.0, GearBox.Gear.D));
        assertThrows(IllegalArgumentException.class, () -> brakeController.apply(50.0, -10.0, GearBox.Gear.D));
    }

    @Test
    void testNullGearThrowsException() {
        assertThrows(NullPointerException.class, () -> brakeController.apply(50.0, 50.0, null));
        assertThrows(NullPointerException.class, () -> brakeController.emergencyBrake(null));
    }

    @ParameterizedTest
    @MethodSource("provideNormalBrakingParameters")
    void testNormalBrakingAndCapping(double requestedPressure, double speedKmh, double expectedPressure) {
        double actual = brakeController.apply(requestedPressure, speedKmh, GearBox.Gear.D);
        
        assertEquals(expectedPressure, actual, TOLERANCE);
        assertEquals(expectedPressure, brakeController.getPressure(), TOLERANCE);
        assertFalse(brakeController.isWarningActive());
    }

    static Stream<Arguments> provideNormalBrakingParameters() {
        return Stream.of(
            Arguments.of(50.0, 20.0, 50.0), 
            Arguments.of(90.0, 20.0, 90.0),
            Arguments.of(50.0, 100.0, 50.0),
            Arguments.of(120.0, 20.0, 100.0)
        );
    }

    @ParameterizedTest
    @CsvSource({
        "80.0, 60.0, 80.0",
        "79.999, 60.0, 79.999",
        "80.001, 60.0, 90.0",
        "95.0, 30.0, 95.0",
        "95.0, 29.999, 95.0",
        "95.0, 30.001, 90.0"
    })
    void testBoundaryValuesForAbs(double requestedPressure, double speedKmh, double expectedPressure) {
        double actual = brakeController.apply(requestedPressure, speedKmh, GearBox.Gear.D);
        assertEquals(expectedPressure, actual, TOLERANCE);
    }

    @Test
    void testAbsBrakingAlternatesPulsesAndDoesNotActivateWarning() {
        double firstPulse = brakeController.apply(100.0, 50.0, GearBox.Gear.D);
        assertEquals(90.0, firstPulse, TOLERANCE);
        assertFalse(brakeController.isWarningActive());

        double secondPulse = brakeController.apply(100.0, 50.0, GearBox.Gear.D);
        assertEquals(60.0, secondPulse, TOLERANCE);

        double thirdPulse = brakeController.apply(100.0, 50.0, GearBox.Gear.D);
        assertEquals(90.0, thirdPulse, TOLERANCE);
    }

    @Test
    void testNormalBrakeResetsAbsCycle() {
        brakeController.apply(100.0, 50.0, GearBox.Gear.D);
        brakeController.apply(20.0, 20.0, GearBox.Gear.D);
        double nextPulse = brakeController.apply(100.0, 50.0, GearBox.Gear.D);
        assertEquals(90.0, nextPulse, TOLERANCE);
    }

    @Test
    void testRejectedParkRequestPreservesAbsPhase() {
        brakeController.apply(100.0, 50.0, GearBox.Gear.D);
        assertThrows(IllegalStateException.class, () -> brakeController.apply(100.0, 50.0, GearBox.Gear.P));
        assertFalse(brakeController.isWarningActive());
        
        double nextPulse = brakeController.apply(100.0, 50.0, GearBox.Gear.D);
        assertEquals(60.0, nextPulse, TOLERANCE);
    }

    @Test
    void testEmergencyBrake() {
        brakeController.apply(100.0, 50.0, GearBox.Gear.D); 
        brakeController.emergencyBrake(GearBox.Gear.D);
        
        assertEquals(100.0, brakeController.getPressure(), TOLERANCE);
        assertTrue(brakeController.isWarningActive());
        
        double pulseAfterEmergency = brakeController.apply(100.0, 50.0, GearBox.Gear.D);
        assertEquals(90.0, pulseAfterEmergency, TOLERANCE);
    }

    @Test
    void testRepeatedEmergencyBrakeKeepsFullPressure() {
        brakeController.emergencyBrake(GearBox.Gear.D);
        brakeController.emergencyBrake(GearBox.Gear.D);
        
        assertEquals(100.0, brakeController.getPressure(), TOLERANCE);
        assertTrue(brakeController.isWarningActive());
    }
}