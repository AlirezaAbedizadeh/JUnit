package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.Arrays;
import java.util.stream.Stream;

import static autopilot.core.GearBox.Gear.D;
import static autopilot.core.GearBox.Gear.N;
import static autopilot.core.GearBox.Gear.P;
import static autopilot.core.GearBox.Gear.R;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.params.provider.Arguments.arguments;

class GearBoxRemainingTest {
    private static final double STOPPED_SPEED_KMH = 0.0;
    private static final double REVERSE_ENTRY_LIMIT_KMH = 5.0;
    private static final String SENSOR_FAILURE_MESSAGE = "Speed reading unavailable";
    private static final double MESSAGE_TEST_SPEED_KMH = 12.5;
    private static final String FORBIDDEN_TRANSITION_MESSAGE = "Cannot change from P to D at 12.5 km/h";

    private static final boolean[][] AT_REST = {
            {true,  true,  false, false},
            {false, true,  true,  true},
            {false, false, true,  true},
            {false, true,  false, true}
    };
    private static final boolean[][] MOVING_BELOW_REVERSE_LIMIT = {
            {true,  true,  false, false},
            {false, true,  true,  false},
            {false, false, true,  true},
            {false, false, false, true}
    };
    private static final boolean[][] AT_REVERSE_LIMIT = {
            {true,  false, false, false},
            {false, true,  true,  false},
            {false, false, true,  true},
            {false, false, false, true}
    };

    @ParameterizedTest(name = "{0} -> {1} at {2} km/h")
    @MethodSource("allowedTransitions")
    void changeTo_allowedTransition_selectsRequestedGear(GearBox.Gear from,
                                                        GearBox.Gear to, double speedKmh) {
        StubSpeedSensor sensor = new StubSpeedSensor();
        GearBox gearBox = inGear(from, sensor);
        sensor.speedKmh = speedKmh;

        gearBox.changeTo(to);

        assertEquals(to, gearBox.getCurrentGear());
    }

    @ParameterizedTest(name = "reject {0} -> {1} at {2} km/h")
    @MethodSource("forbiddenTransitions")
    void changeTo_forbiddenTransition_throwsIllegalGearChange(GearBox.Gear from,
                                                            GearBox.Gear to, double speedKmh) {
        StubSpeedSensor sensor = new StubSpeedSensor();
        GearBox gearBox = inGear(from, sensor);
        sensor.speedKmh = speedKmh;

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(to));

        assertEquals(from, gearBox.getCurrentGear());
    }

    static Stream<Arguments> allowedTransitions() {
        return Stream.concat(transitions(true), Stream.of(
                arguments(P, R, -0.0), arguments(R, D, -0.0), arguments(D, R, -0.0),
                arguments(R, N, Double.MAX_VALUE), arguments(N, D, Double.MAX_VALUE)));
    }

    static Stream<Arguments> forbiddenTransitions() {
        return transitions(false);
    }

    private static Stream<Arguments> transitions(boolean allowed) {
        return Stream.of(
                new TransitionPartition(STOPPED_SPEED_KMH, AT_REST),
                new TransitionPartition(Math.nextDown(REVERSE_ENTRY_LIMIT_KMH), MOVING_BELOW_REVERSE_LIMIT),
                new TransitionPartition(REVERSE_ENTRY_LIMIT_KMH, AT_REVERSE_LIMIT)
        ).flatMap(partition -> Arrays.stream(GearBox.Gear.values()).flatMap(from ->
                Arrays.stream(GearBox.Gear.values())
                        .filter(to -> to != from)
                        .filter(to -> !(from == P && to == R && partition.speedKmh() == STOPPED_SPEED_KMH))
                        .filter(to -> partition.allowed()[from.ordinal()][to.ordinal()] == allowed)
                        .map(to -> arguments(from, to, partition.speedKmh()))));
    }

    @ParameterizedTest(name = "P -> R at boundary speed {0}")
    @MethodSource("extraReverseEntrySpeeds")
    void changeTo_reverseEntryOutsideLimit_throwsIllegalGearChange(double speedKmh) {
        GearBox gearBox = new GearBox(() -> speedKmh);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(R));

        assertEquals(P, gearBox.getCurrentGear());
    }

    static Stream<Double> extraReverseEntrySpeeds() {
        return Stream.of(Math.nextUp(REVERSE_ENTRY_LIMIT_KMH), Double.MAX_VALUE);
    }

    @ParameterizedTest(name = "{0} -> {1} at smallest positive speed")
    @MethodSource("directionChanges")
    void changeTo_nonzeroDirectionChange_throwsIllegalGearChange(GearBox.Gear from,
                                                               GearBox.Gear to) {
        StubSpeedSensor sensor = new StubSpeedSensor();
        GearBox gearBox = inGear(from, sensor);
        sensor.speedKmh = Double.MIN_VALUE;

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(to));

        assertEquals(from, gearBox.getCurrentGear());
    }

    static Stream<Arguments> directionChanges() {
        return Stream.of(arguments(R, D), arguments(D, R));
    }

    @ParameterizedTest(name = "same gear {0} does not need a speed reading")
    @MethodSource("nonParkGears")
    void changeTo_sameGear_doesNotReadSpeedSensor(GearBox.Gear gear) {
        GearBox gearBox = inGear(gear, new StubSpeedSensor());
        gearBox.setSpeedSensor(() -> {
            throw new AssertionError("An unchanged gear must not read the speed sensor");
        });

        gearBox.changeTo(gear);

        assertEquals(gear, gearBox.getCurrentGear());
    }

    static Stream<GearBox.Gear> nonParkGears() {
        return Stream.of(R, N, D);
    }

    static Stream<GearBox.Gear> allGears() {
        return Arrays.stream(GearBox.Gear.values());
    }

    @ParameterizedTest(name = "invalid speed {2} for {0} -> {1}")
    @MethodSource("invalidSpeedChanges")
    void changeTo_invalidSpeed_throwsIllegalArgument(GearBox.Gear from,
                                                   GearBox.Gear to, double invalidSpeed) {
        StubSpeedSensor sensor = new StubSpeedSensor();
        GearBox gearBox = inGear(from, sensor);
        sensor.speedKmh = invalidSpeed;

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(to));

        assertEquals(from, gearBox.getCurrentGear());
    }

    static Stream<Arguments> invalidSpeedChanges() {
        return Stream.of(new GearBox.Gear[]{P, R}, new GearBox.Gear[]{R, N},
                        new GearBox.Gear[]{N, D}, new GearBox.Gear[]{D, R})
                .flatMap(pair -> TestInputs.invalidNonNegativeNumbers()
                        .filter(speed -> !(pair[0] == P && speed == -1.0))
                        .mapToObj(speed -> arguments(pair[0], pair[1], speed)));
    }

    @ParameterizedTest(name = "null target while in {0}")
    @MethodSource("allGears")
    void changeTo_nullTarget_throwsBeforeReadingSensorAndPreservesGear(GearBox.Gear from) {
        GearBox gearBox = inGear(from, new StubSpeedSensor());
        gearBox.setSpeedSensor(() -> {
            throw new AssertionError("A null target must be rejected before reading speed");
        });

        assertThrows(NullPointerException.class, () -> gearBox.changeTo(null));

        assertEquals(from, gearBox.getCurrentGear());
    }

    @ParameterizedTest(name = "sensor failure during {0} -> {1}")
    @MethodSource("sensorFailureTransitions")
    void changeTo_failedSpeedSensor_preservesGearAndPropagatesOriginalFailure(GearBox.Gear from,
                                                                           GearBox.Gear to) {
        SensorFailureException failure = new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        GearBox gearBox = inGear(from, new StubSpeedSensor());
        gearBox.setSpeedSensor(() -> {
            throw failure;
        });

        assertSame(failure, assertThrows(SensorFailureException.class, () -> gearBox.changeTo(to)));

        assertEquals(from, gearBox.getCurrentGear());
    }

    static Stream<Arguments> sensorFailureTransitions() {
        return Stream.of(arguments(P, R), arguments(R, N), arguments(N, D), arguments(D, R));
    }

    @Test
    void changeTo_forbiddenTransition_reportsContextInException() {
        GearBox gearBox = new GearBox(() -> MESSAGE_TEST_SPEED_KMH);

        IllegalGearChangeException failure = assertThrows(IllegalGearChangeException.class,
                () -> gearBox.changeTo(D));

        assertEquals(FORBIDDEN_TRANSITION_MESSAGE, failure.getMessage());
    }

    @Test
    void changeTo_invalidSpeed_doesNotChangeGearBeforeRecovery() {
        StubSpeedSensor sensor = new StubSpeedSensor();
        GearBox gearBox = inGear(R, sensor);
        sensor.speedKmh = Double.NaN;
        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(D));
        sensor.speedKmh = STOPPED_SPEED_KMH;

        gearBox.changeTo(D);

        assertEquals(D, gearBox.getCurrentGear());
    }

    private static GearBox inGear(GearBox.Gear gear, StubSpeedSensor sensor) {
        GearBox gearBox = new GearBox(sensor);
        if (gear != P) {
            gearBox.changeTo(R);
            if (gear == N || gear == D) {
                gearBox.changeTo(gear);
            }
        }
        return gearBox;
    }

    private record TransitionPartition(double speedKmh, boolean[][] allowed) {
    }

    private static final class StubSpeedSensor implements SpeedSensor {
        private double speedKmh = STOPPED_SPEED_KMH;

        @Override
        public double getSpeedKmh() {
            return speedKmh;
        }
    }
}
