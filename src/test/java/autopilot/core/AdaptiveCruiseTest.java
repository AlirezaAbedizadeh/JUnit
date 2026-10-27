package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static autopilot.core.AdaptiveCruise.Mode.ACCELERATE;
import static autopilot.core.AdaptiveCruise.Mode.BRAKE;
import static autopilot.core.AdaptiveCruise.Mode.EMERGENCY_BRAKE;
import static autopilot.core.AdaptiveCruise.Mode.HOLD;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.params.provider.Arguments.arguments;

class AdaptiveCruiseTest {
    private static final double TARGET_SPEED_KMH = 100.0;
    private static final double CRUISING_SPEED_KMH = 80.0;
    private static final double CLEAR_DISTANCE_METERS = 100.0;
    private static final double BRAKING_DISTANCE_METERS = 30.0;
    private static final double EMERGENCY_DISTANCE_METERS = 20.0;
    private static final double SAFE_DISTANCE_METERS = 40.0;
    private static final double RELEASE_DISTANCE_METERS = 48.0;
    private static final double WITHIN_HYSTERESIS_DISTANCE_METERS = 44.0;
    private static final double MAX_ROUNDING_ULPS = 4.0;
    private static final String SPEED_VALIDATION_MESSAGE = "speed must be a finite non-negative value";

    @ParameterizedTest(name = "speed {0} km/h has half-speed following distance")
    @ValueSource(doubles = {0.0, -0.0, Double.MIN_VALUE, Double.MIN_NORMAL,
            0.5, 1.0, 5.0, 79.5, 80.0, 120.0, 200.0, Double.MAX_VALUE})
    void safeDistanceFor_validSpeed_returnsHalfSpeedInMeters(double speedKmh) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        double actualDistance = cruise.safeDistanceFor(speedKmh);

        assertEquals(speedKmh / 2.0, actualDistance, 0.0);
    }

    @ParameterizedTest(name = "invalid speed: {0}")
    @MethodSource("autopilot.core.TestInputs#invalidNonNegativeNumbers")
    void safeDistanceFor_invalidSpeed_throwsWithoutChangingBrakingState(double invalidSpeed) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(CRUISING_SPEED_KMH, BRAKING_DISTANCE_METERS);

        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(invalidSpeed));

        assertEquals(decision(BRAKE, 10.0), cruise.update(CRUISING_SPEED_KMH, SAFE_DISTANCE_METERS));
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("freshDecisions")
    void update_initialState_returnsExpectedDecision(String scenario, double targetSpeed,
                                                     double speed, double distance,
                                                     AdaptiveCruise.Decision expected) {
        AdaptiveCruise cruise = new AdaptiveCruise(targetSpeed);

        AdaptiveCruise.Decision actual = cruise.update(speed, distance);

        assertEquals(expected, actual, scenario);
    }

    static Stream<Arguments> freshDecisions() {
        return Stream.of(
                arguments("signed-zero speed and distance are emergency", 100.0, -0.0, -0.0,
                        decision(EMERGENCY_BRAKE, 100.0)),
                arguments("zero target and clear road hold at rest", 0.0, 0.0, 1.0,
                        decision(HOLD, 0.0)),
                arguments("start from rest with a clear road", 100.0, 0.0, 1.0,
                        decision(ACCELERATE, 5.0)),
                arguments("smallest positive target retains its acceleration", Double.MIN_VALUE, 0.0, 1.0,
                        decision(ACCELERATE, Double.MIN_VALUE)),
                arguments("subnormal speed and distance remain valid", 100.0,
                        Double.MIN_VALUE, Double.MIN_VALUE, decision(ACCELERATE, 5.0)),
                arguments("obstacle at zero distance", 100.0, 80.0, 0.0,
                        decision(EMERGENCY_BRAKE, 100.0)),
                arguments("just below emergency boundary", 100.0, 80.0, Math.nextDown(20.0),
                        decision(EMERGENCY_BRAKE, 100.0)),
                arguments("exact emergency boundary is inclusive", 100.0, 80.0, 20.0,
                        decision(EMERGENCY_BRAKE, 100.0)),
                arguments("ordinary proportional braking", 100.0, 80.0, 30.0,
                        decision(BRAKE, 25.0)),
                arguments("low speed retains proportional braking", 100.0, 2.0, 0.75,
                        decision(BRAKE, 25.0)),
                arguments("fractional distance retains pressure precision", 100.0, 80.0, 31.5,
                        decision(BRAKE, 21.25)),
                arguments("fractional speed retains safe-distance precision", 100.0, 79.5, 29.8125,
                        decision(BRAKE, 25.0)),
                arguments("safe distance is hold, even below target", 100.0, 80.0, 40.0,
                        decision(HOLD, 0.0)),
                arguments("just above safe distance accelerates", 100.0, 80.0, Math.nextUp(40.0),
                        decision(ACCELERATE, 5.0)),
                arguments("large speed deficit is capped", 100.0, 80.0, 100.0,
                        decision(ACCELERATE, 5.0)),
                arguments("deficit exactly one acceleration step", 100.0, 95.0, 100.0,
                        decision(ACCELERATE, 5.0)),
                arguments("small deficit reaches target without overshoot", 100.0, 98.0, 100.0,
                        decision(ACCELERATE, 2.0)),
                arguments("fractional target deficit is retained", 99.75, 98.5, 100.0,
                        decision(ACCELERATE, 1.25)),
                arguments("target reached with clear road", 100.0, 100.0, 100.0,
                        decision(HOLD, 0.0)),
                arguments("speed above target holds with clear road", 100.0, 105.0, 100.0,
                        decision(HOLD, 0.0)),
                arguments("zero target still brakes for an obstacle", 0.0, 80.0, 30.0,
                        decision(BRAKE, 25.0)),
                arguments("zero target does not suppress emergency braking", 0.0, 80.0, 20.0,
                        decision(EMERGENCY_BRAKE, 100.0)),
                arguments("largest finite speed and distance remain valid", Double.MAX_VALUE,
                        Double.MAX_VALUE, Double.MAX_VALUE, decision(HOLD, 0.0))
        );
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("proportionalDecisions")
    void update_distanceBelowSafeDistance_returnsProportionalBrakeDecision(String scenario,
                                                                         double distance,
                                                                         double expectedPressure) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        AdaptiveCruise.Decision actual = cruise.update(CRUISING_SPEED_KMH, distance);

        assertAll(scenario,
                () -> assertEquals(BRAKE, actual.mode()),
                () -> assertEquals(expectedPressure, actual.amount(),
                        MAX_ROUNDING_ULPS * Math.ulp(expectedPressure)));
    }

    static Stream<Arguments> proportionalDecisions() {
        return Stream.of(
                arguments("just above emergency changes to ordinary brake", Math.nextUp(20.0), 50.0),
                arguments("just below safe distance applies a tiny brake", Math.nextDown(40.0),
                        1.7763568394002505e-14)
        );
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("brakingHistories")
    void update_previousBraking_respectsHysteresis(String scenario, double startingDistance,
                                                  double nextSpeed, double nextDistance,
                                                  AdaptiveCruise.Decision expected) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(CRUISING_SPEED_KMH, startingDistance);

        AdaptiveCruise.Decision actual = cruise.update(nextSpeed, nextDistance);

        assertEquals(expected, actual, scenario);
    }

    static Stream<Arguments> brakingHistories() {
        return Stream.of(BRAKING_DISTANCE_METERS, EMERGENCY_DISTANCE_METERS)
                .flatMap(start -> Stream.of(
                        arguments("continued proportional brake after " + start, start, 80.0, 35.0,
                                decision(BRAKE, 12.5)),
                        arguments("at safe distance stay braking after " + start, start, 80.0, 40.0,
                                decision(BRAKE, 10.0)),
                        arguments("above safe distance stay braking after " + start, start, 80.0, 44.0,
                                decision(BRAKE, 10.0)),
                        arguments("just below release stay braking after " + start, start, 80.0,
                                Math.nextDown(RELEASE_DISTANCE_METERS), decision(BRAKE, 10.0)),
                        arguments("at release braking ends after " + start, start, 80.0, 48.0,
                                decision(ACCELERATE, 5.0)),
                        arguments("above release braking ends after " + start, start, 80.0,
                                Math.nextUp(RELEASE_DISTANCE_METERS), decision(ACCELERATE, 5.0)),
                        arguments("emergency overrides hysteresis after " + start, start, 80.0, 20.0,
                                decision(EMERGENCY_BRAKE, 100.0)),
                        arguments("repeated emergency remains emergency after " + start, start, 80.0, 0.0,
                                decision(EMERGENCY_BRAKE, 100.0)),
                        arguments("release at target holds after " + start, start, 100.0, 60.0,
                                decision(HOLD, 0.0)),
                        arguments("release above target holds after " + start, start, 120.0, 72.0,
                                decision(HOLD, 0.0)),
                        arguments("release threshold follows new speed after " + start, start, 40.0, 24.0,
                                decision(ACCELERATE, 5.0)),
                        arguments("stopped with clear space releases brake after " + start, start, 0.0, 1.0,
                                decision(ACCELERATE, 5.0)),
                        arguments("stopped at obstacle remains emergency after " + start, start, 0.0, 0.0,
                                decision(EMERGENCY_BRAKE, 100.0))
                ));
    }

    @ParameterizedTest(name = "release after brake distance {0}")
    @MethodSource("initialBrakeDistances")
    void update_releasedHysteresis_doesNotKeepBrakingAtSafeDistance(double startingDistance) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(CRUISING_SPEED_KMH, startingDistance);
        cruise.update(CRUISING_SPEED_KMH, RELEASE_DISTANCE_METERS);

        AdaptiveCruise.Decision actual = cruise.update(CRUISING_SPEED_KMH, SAFE_DISTANCE_METERS);

        assertEquals(decision(HOLD, 0.0), actual);
    }

    static Stream<Double> initialBrakeDistances() {
        return Stream.of(BRAKING_DISTANCE_METERS, EMERGENCY_DISTANCE_METERS);
    }

    @Test
    void update_previousHold_doesNotCreateBrakingHysteresis() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(CRUISING_SPEED_KMH, SAFE_DISTANCE_METERS);

        AdaptiveCruise.Decision actual = cruise.update(CRUISING_SPEED_KMH,
                WITHIN_HYSTERESIS_DISTANCE_METERS);

        assertEquals(decision(ACCELERATE, 5.0), actual);
    }

    @ParameterizedTest(name = "invalid speed {0}, braking {1}")
    @MethodSource("invalidNumbersAndBrakingStates")
    void update_invalidSpeed_throwsAndPreservesBrakingState(double invalidSpeed, boolean braking,
                                                          AdaptiveCruise.Decision expected) {
        AdaptiveCruise cruise = withBrakingState(braking);

        assertThrows(IllegalArgumentException.class,
                () -> cruise.update(invalidSpeed, CLEAR_DISTANCE_METERS));

        assertEquals(expected, cruise.update(CRUISING_SPEED_KMH, SAFE_DISTANCE_METERS));
    }

    @ParameterizedTest(name = "invalid distance {0}, braking {1}")
    @MethodSource("invalidNumbersAndBrakingStates")
    void update_invalidDistance_throwsAndPreservesBrakingState(double invalidDistance, boolean braking,
                                                             AdaptiveCruise.Decision expected) {
        AdaptiveCruise cruise = withBrakingState(braking);

        assertThrows(IllegalArgumentException.class,
                () -> cruise.update(CRUISING_SPEED_KMH, invalidDistance));

        assertEquals(expected, cruise.update(CRUISING_SPEED_KMH, SAFE_DISTANCE_METERS));
    }

    static Stream<Arguments> invalidNumbersAndBrakingStates() {
        return TestInputs.invalidNonNegativeNumbers().boxed().flatMap(invalid -> Stream.of(
                arguments(invalid, false, decision(HOLD, 0.0)),
                arguments(invalid, true, decision(BRAKE, 10.0))
        ));
    }

    @ParameterizedTest(name = "invalid speed {0} and invalid distance {1}")
    @MethodSource("multipleInvalidInputs")
    void update_multipleInvalidInputs_reportsSpeedFailureFirst(double speed, double distance) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                () -> cruise.update(speed, distance));

        assertEquals(SPEED_VALIDATION_MESSAGE, failure.getMessage());
    }

    static Stream<Arguments> multipleInvalidInputs() {
        return Stream.of(arguments(-1.0, -1.0), arguments(-1.0, Double.NaN),
                arguments(Double.NaN, -1.0), arguments(Double.NaN, Double.NaN));
    }

    private static AdaptiveCruise withBrakingState(boolean braking) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        if (braking) {
            cruise.update(CRUISING_SPEED_KMH, BRAKING_DISTANCE_METERS);
        }
        return cruise;
    }

    private static AdaptiveCruise.Decision decision(AdaptiveCruise.Mode mode, double amount) {
        return new AdaptiveCruise.Decision(mode, amount);
    }
}
