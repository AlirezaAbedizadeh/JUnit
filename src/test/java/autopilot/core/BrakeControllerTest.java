package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;
import java.util.function.Consumer;

import static autopilot.core.GearBox.Gear.D;
import static autopilot.core.GearBox.Gear.P;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.params.provider.Arguments.arguments;

class BrakeControllerTest {
    private static final double ABS_REQUEST_PRESSURE = 95.0;
    private static final double ABS_SPEED_KMH = 60.0;
    private static final double ORDINARY_PRESSURE = 25.0;
    private static final double LOW_SPEED_KMH = 20.0;
    private static final double MAX_PRESSURE = 100.0;
    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;
    private static final String PRESSURE_VALIDATION_MESSAGE = "pressure must be a finite non-negative value";
    private static final String SPEED_VALIDATION_MESSAGE = "speed must be a finite non-negative value";

    @ParameterizedTest(name = "{0}")
    @CsvSource(textBlock = """
            zeroPressureAndSignedZeroSpeed,0.0,               -0.0,               D, 0.0
            negativeZeroPressure,        -0.0,                0.0,               D, 0.0
            ordinaryPressure,            50.0,               20.0,               D, 50.0
            highPressureWithoutAbs,      90.0,               20.0,               D, 90.0
            moderatePressureAtHighSpeed, 50.0,              100.0,               D, 50.0
            pressureAtAbsBoundary,       80.0,               60.0,               D, 80.0
            pressureBelowAbsBoundary,    79.99999999999999,   60.0,               D, 79.99999999999999
            pressureAboveAbsBoundary,    80.00000000000001,   60.0,               D, 90.0
            speedAtAbsBoundary,          95.0,               30.0,               D, 95.0
            speedBelowAbsBoundary,       95.0,               29.999999999999996,  D, 95.0
            speedAboveAbsBoundary,       95.0,               30.000000000000004,  D, 90.0
            bothAtBoundary,              80.0,               30.0,               D, 80.0
            pressureBelowMaximum,        99.99999999999999,   20.0,               D, 99.99999999999999
            fullPressureWithoutAbs,     100.0,               20.0,               D, 100.0
            pressureAboveMaximum,       100.00000000000001,   20.0,               D, 100.0
            excessivePressureIsCapped,  120.0,               20.0,               D, 100.0
            excessivePressureWithAbs,   150.0,               60.0,               D, 90.0
            largestPressureIsCapped,      1.7976931348623157E308, 0.0,            D, 100.0
            largestSpeedEnablesAbs,      95.0,                1.7976931348623157E308, D, 90.0
            smallestPressureIsValid,      4.9E-324,            4.9E-324,          D, 4.9E-324
            reverseAllowsBraking,        25.0,               20.0,               R, 25.0
            neutralAllowsBraking,        25.0,               20.0,               N, 25.0
            reverseAllowsAbs,            95.0,               60.0,               R, 90.0
            neutralAllowsAbs,            95.0,               60.0,               N, 90.0
            """)
    void apply_validRequest_returnsAndStoresExpectedBrakeState(String scenario, double requestedPressure,
                                                              double speedKmh, GearBox.Gear gear,
                                                              double expectedPressure) {
        BrakeController brakes = new BrakeController();

        double actualPressure = brakes.apply(requestedPressure, speedKmh, gear);

        assertAll(scenario,
                () -> assertEquals(expectedPressure, actualPressure, 0.0),
                () -> assertEquals(expectedPressure, brakes.getPressure(), 0.0),
                () -> assertFalse(brakes.isWarningActive()));
    }

    @Test
    void apply_repeatedAbsRequests_alternatesAndStoresPulses() {
        BrakeController brakes = new BrakeController();

        Pulse[] actual = {absPulse(brakes), absPulse(brakes), absPulse(brakes), absPulse(brakes)};

        assertArrayEquals(new Pulse[]{
                new Pulse(ABS_HIGH_PULSE, ABS_HIGH_PULSE, false),
                new Pulse(ABS_LOW_PULSE, ABS_LOW_PULSE, false),
                new Pulse(ABS_HIGH_PULSE, ABS_HIGH_PULSE, false),
                new Pulse(ABS_LOW_PULSE, ABS_LOW_PULSE, false)
        }, actual);
    }

    @ParameterizedTest(name = "pressure {0}, speed {1}, warning {2}, next high pulse {3}")
    @MethodSource("nonAbsRequests")
    void apply_nonAbsInterruption_restartsAbsWithHighPulse(double pressure, double speed,
                                                         boolean warning, boolean nextHigh) {
        BrakeController brakes = withAbsState(warning, nextHigh);

        brakes.apply(pressure, speed, D);

        assertEquals(new AbsObservation(pressure, warning,
                        new Pulse(ABS_HIGH_PULSE, ABS_HIGH_PULSE, warning)),
                nextAbsObservation(brakes));
    }

    static Stream<Arguments> nonAbsRequests() {
        return Stream.of(new NonAbsRequest(0.0, ABS_SPEED_KMH),
                        new NonAbsRequest(80.0, ABS_SPEED_KMH),
                        new NonAbsRequest(ABS_REQUEST_PRESSURE, 30.0),
                        new NonAbsRequest(ORDINARY_PRESSURE, LOW_SPEED_KMH))
                .flatMap(request -> Stream.of(false, true).flatMap(warning ->
                        Stream.of(false, true).map(nextHigh ->
                                arguments(request.pressure(), request.speed(), warning, nextHigh))));
    }

    @ParameterizedTest(name = "invalid pressure {0}")
    @MethodSource("autopilot.core.TestInputs#invalidNonNegativeNumbers")
    void apply_invalidPressure_throwsWithoutChangingAbsState(double invalidPressure) {
        BrakeController brakes = afterFirstAbsPulse();

        assertThrows(IllegalArgumentException.class,
                () -> brakes.apply(invalidPressure, LOW_SPEED_KMH, D));

        assertEquals(preservedAbsState(), nextAbsObservation(brakes));
    }

    @ParameterizedTest(name = "invalid speed {0}")
    @MethodSource("autopilot.core.TestInputs#invalidNonNegativeNumbers")
    void apply_invalidSpeed_throwsWithoutChangingAbsState(double invalidSpeed) {
        BrakeController brakes = afterFirstAbsPulse();

        assertThrows(IllegalArgumentException.class,
                () -> brakes.apply(ORDINARY_PRESSURE, invalidSpeed, D));

        assertEquals(preservedAbsState(), nextAbsObservation(brakes));
    }

    @ParameterizedTest(name = "park rejects pressure {0}, speed {1}")
    @MethodSource("requestsRejectedByGear")
    void apply_parkGear_throwsWithoutChangingAbsState(double pressure, double speed) {
        BrakeController brakes = afterFirstAbsPulse();

        assertThrows(IllegalStateException.class,
                () -> brakes.apply(pressure, speed, P));

        assertEquals(preservedAbsState(), nextAbsObservation(brakes));
    }

    @ParameterizedTest(name = "null gear rejects pressure {0}, speed {1}")
    @MethodSource("requestsRejectedByGear")
    void apply_nullGear_throwsWithoutChangingAbsState(double pressure, double speed) {
        BrakeController brakes = afterFirstAbsPulse();

        assertThrows(NullPointerException.class,
                () -> brakes.apply(pressure, speed, null));

        assertEquals(preservedAbsState(), nextAbsObservation(brakes));
    }

    static Stream<Arguments> requestsRejectedByGear() {
        return Stream.of(arguments(0.0, 0.0), arguments(ORDINARY_PRESSURE, LOW_SPEED_KMH),
                arguments(ABS_REQUEST_PRESSURE, ABS_SPEED_KMH));
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("rejectedRequestsAndPreviousStates")
    void apply_rejectedRequests_preserveOtherReachableAbsStates(String scenario, RejectedRequest request,
                                                              AbsStateSpec previousState) {
        BrakeController brakes = withAbsState(previousState.warningActive(), previousState.nextHigh());

        assertThrows(request.failureType(), () -> request.operation().accept(brakes));

        assertEquals(previousState.expected(), nextAbsObservation(brakes), scenario);
    }

    static Stream<Arguments> rejectedRequestsAndPreviousStates() {
        return rejectedRequests().flatMap(request -> additionalRejectionStates().map(state ->
                arguments(request.name() + ", warning " + state.warningActive()
                        + ", next high pulse " + state.nextHigh(), request, state)));
    }

    private static Stream<RejectedRequest> rejectedRequests() {
        return Stream.of(
                new RejectedRequest("negative pressure", IllegalArgumentException.class,
                        brakes -> brakes.apply(-1.0, LOW_SPEED_KMH, D)),
                new RejectedRequest("non-finite pressure", IllegalArgumentException.class,
                        brakes -> brakes.apply(Double.NaN, LOW_SPEED_KMH, D)),
                new RejectedRequest("negative speed", IllegalArgumentException.class,
                        brakes -> brakes.apply(ORDINARY_PRESSURE, -1.0, D)),
                new RejectedRequest("non-finite speed", IllegalArgumentException.class,
                        brakes -> brakes.apply(ORDINARY_PRESSURE, Double.NaN, D)),
                new RejectedRequest("park braking", IllegalStateException.class,
                        brakes -> brakes.apply(ORDINARY_PRESSURE, LOW_SPEED_KMH, P)),
                new RejectedRequest("null braking gear", NullPointerException.class,
                        brakes -> brakes.apply(ORDINARY_PRESSURE, LOW_SPEED_KMH, null)),
                new RejectedRequest("park emergency", IllegalStateException.class,
                        brakes -> brakes.emergencyBrake(P)),
                new RejectedRequest("null emergency gear", NullPointerException.class,
                        brakes -> brakes.emergencyBrake(null))
        );
    }

    private static Stream<AbsStateSpec> additionalRejectionStates() {
        return Stream.of(
                new AbsStateSpec(false, true, new AbsObservation(ABS_LOW_PULSE, false,
                        new Pulse(ABS_HIGH_PULSE, ABS_HIGH_PULSE, false))),
                new AbsStateSpec(true, false, new AbsObservation(ABS_HIGH_PULSE, true,
                        new Pulse(ABS_LOW_PULSE, ABS_LOW_PULSE, true))),
                new AbsStateSpec(true, true, new AbsObservation(ABS_LOW_PULSE, true,
                        new Pulse(ABS_HIGH_PULSE, ABS_HIGH_PULSE, true)))
        );
    }

    @ParameterizedTest(name = "invalid pressure {0}, speed {1}, gear {2}")
    @MethodSource("invalidPressureAndOtherInputs")
    void apply_multipleInvalidInputs_reportsPressureFailureFirst(double pressure, double speed,
                                                               GearBox.Gear gear) {
        BrakeController brakes = new BrakeController();

        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                () -> brakes.apply(pressure, speed, gear));

        assertEquals(PRESSURE_VALIDATION_MESSAGE, failure.getMessage());
    }

    static Stream<Arguments> invalidPressureAndOtherInputs() {
        return Stream.of(arguments(-1.0, -1.0, P), arguments(-1.0, Double.NaN, null),
                arguments(Double.NaN, -1.0, null), arguments(Double.NaN, Double.NaN, P));
    }

    @ParameterizedTest(name = "invalid speed {0}, gear {1}")
    @MethodSource("invalidSpeedAndGear")
    void apply_invalidSpeedAndGear_reportsSpeedFailureFirst(double speed, GearBox.Gear gear) {
        BrakeController brakes = new BrakeController();

        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                () -> brakes.apply(ORDINARY_PRESSURE, speed, gear));

        assertEquals(SPEED_VALIDATION_MESSAGE, failure.getMessage());
    }

    static Stream<Arguments> invalidSpeedAndGear() {
        return Stream.of(arguments(-1.0, P), arguments(-1.0, null),
                arguments(Double.NaN, P), arguments(Double.NaN, null));
    }

    @ParameterizedTest(name = "pressure {0} after emergency keeps warning active")
    @CsvSource({"0.0, 20.0", "25.0, 20.0"})
    void apply_afterEmergency_preservesActiveWarning(double pressure, double speedKmh) {
        BrakeController brakes = new BrakeController();
        brakes.emergencyBrake(D);

        double actualPressure = brakes.apply(pressure, speedKmh, D);

        assertAll(
                () -> assertEquals(pressure, actualPressure, 0.0),
                () -> assertEquals(pressure, brakes.getPressure(), 0.0),
                () -> assertTrue(brakes.isWarningActive()));
    }

    @ParameterizedTest(name = "emergency state in {0}")
    @EnumSource(value = GearBox.Gear.class, names = {"R", "N", "D"})
    void emergencyBrake_nonParkGear_appliesFullPressureAndWarning(GearBox.Gear gear) {
        BrakeController brakes = new BrakeController();

        brakes.emergencyBrake(gear);

        assertEquals(fullEmergencyState(), nextAbsObservation(brakes));
    }

    @Test
    void emergencyBrake_parkGear_throwsWithoutChangingAbsState() {
        BrakeController brakes = afterFirstAbsPulse();

        assertThrows(IllegalStateException.class, () -> brakes.emergencyBrake(P));

        assertEquals(preservedAbsState(), nextAbsObservation(brakes));
    }

    @Test
    void emergencyBrake_nullGear_throwsWithoutChangingAbsState() {
        BrakeController brakes = afterFirstAbsPulse();

        assertThrows(NullPointerException.class, () -> brakes.emergencyBrake(null));

        assertEquals(preservedAbsState(), nextAbsObservation(brakes));
    }

    @ParameterizedTest(name = "emergency after ABS with previous warning {0}")
    @CsvSource({"false", "true"})
    void emergencyBrake_previousAbsPulse_restoresFullEmergencyState(boolean warning) {
        BrakeController brakes = withAbsState(warning, false);

        brakes.emergencyBrake(D);

        assertEquals(fullEmergencyState(), nextAbsObservation(brakes));
    }

    @Test
    void emergencyBrake_repeatedEmergency_preservesEmergencyState() {
        BrakeController brakes = new BrakeController();
        brakes.emergencyBrake(D);

        brakes.emergencyBrake(D);

        assertEquals(fullEmergencyState(), nextAbsObservation(brakes));
    }

    private static BrakeController afterFirstAbsPulse() {
        return withAbsState(false, false);
    }

    private static BrakeController withAbsState(boolean warning, boolean nextHigh) {
        BrakeController brakes = new BrakeController();
        if (warning) {
            brakes.emergencyBrake(D);
        }
        brakes.apply(ABS_REQUEST_PRESSURE, ABS_SPEED_KMH, D);
        if (nextHigh) {
            brakes.apply(ABS_REQUEST_PRESSURE, ABS_SPEED_KMH, D);
        }
        return brakes;
    }

    private static Pulse absPulse(BrakeController brakes) {
        double actualPressure = brakes.apply(ABS_REQUEST_PRESSURE, ABS_SPEED_KMH, D);
        return new Pulse(actualPressure, brakes.getPressure(), brakes.isWarningActive());
    }

    private static AbsObservation nextAbsObservation(BrakeController brakes) {
        double storedPressure = brakes.getPressure();
        boolean warning = brakes.isWarningActive();
        return new AbsObservation(storedPressure, warning, absPulse(brakes));
    }

    private static AbsObservation preservedAbsState() {
        return new AbsObservation(ABS_HIGH_PULSE, false,
                new Pulse(ABS_LOW_PULSE, ABS_LOW_PULSE, false));
    }

    private static AbsObservation fullEmergencyState() {
        return new AbsObservation(MAX_PRESSURE, true,
                new Pulse(ABS_HIGH_PULSE, ABS_HIGH_PULSE, true));
    }

    private record NonAbsRequest(double pressure, double speed) {
    }

    private record RejectedRequest(String name, Class<? extends RuntimeException> failureType,
                                   Consumer<BrakeController> operation) {
    }

    private record AbsStateSpec(boolean warningActive, boolean nextHigh, AbsObservation expected) {
    }

    private record Pulse(double returnedPressure, double storedPressure, boolean warningActive) {
    }

    private record AbsObservation(double storedPressure, boolean warningActive, Pulse nextPulse) {
    }
}
