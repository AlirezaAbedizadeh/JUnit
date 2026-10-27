package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static autopilot.core.TestInputs.TOLERANCE;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.params.provider.Arguments.arguments;

class SteeringControllerTest {
    private static final double NORMAL_SPEED_KMH = 80.0;
    private static final double HIGH_SPEED_KMH = 121.0;
    private static final double HIGH_SPEED_BOUNDARY_KMH = 120.0;
    private static final double HIGH_SPEED_LIMIT_DEGREES = 90.0;
    private static final double MAX_STEP_DEGREES = 60.0;
    private static final double PHYSICAL_LIMIT_DEGREES = 450.0;
    private static final int TICKS_TO_PHYSICAL_LIMIT = 8;
    private static final String ANGLE_VALIDATION_MESSAGE = "requested angle must be finite";

    @ParameterizedTest(name = "{0}")
    @CsvFileSource(resources = "/autopilot/core/steering-cases.csv", numLinesToSkip = 1)
    void update_validRequest_respectsTargetLimitAndTickRate(String scenario,
                                                           double requestedAngle, double speedKmh,
                                                           int ticks, double expectedTarget,
                                                           double expectedAngle) {
        SteeringController steering = new SteeringController();
        double angle = advance(steering, requestedAngle, speedKmh, ticks);

        SteeringObservation actual = new SteeringObservation(angle,
                steering.getCurrentAngleDegrees(), steering.getTargetAngleDegrees());

        assertEquals(new SteeringObservation(expectedAngle, expectedAngle, expectedTarget), actual, scenario);
    }

    @ParameterizedTest(name = "at speed {0}, target {1}")
    @MethodSource("speedBoundaryCases")
    void update_speedAroundHighSpeedBoundary_selectsCorrectLimit(double speedKmh,
                                                               double requestedAngle,
                                                               double expectedTarget) {
        SteeringController steering = new SteeringController();

        steering.update(requestedAngle, speedKmh);

        assertEquals(expectedTarget, steering.getTargetAngleDegrees());
    }

    static Stream<Arguments> speedBoundaryCases() {
        return Stream.of(
                arguments(Math.nextDown(HIGH_SPEED_BOUNDARY_KMH), 450.0, 450.0),
                arguments(HIGH_SPEED_BOUNDARY_KMH, 450.0, 450.0),
                arguments(Math.nextUp(HIGH_SPEED_BOUNDARY_KMH), 450.0, 90.0),
                arguments(Math.nextDown(HIGH_SPEED_BOUNDARY_KMH), -450.0, -450.0),
                arguments(HIGH_SPEED_BOUNDARY_KMH, -450.0, -450.0),
                arguments(Math.nextUp(HIGH_SPEED_BOUNDARY_KMH), -450.0, -90.0)
        );
    }

    @ParameterizedTest(name = "requested angle {0} gives first tick {1}")
    @MethodSource("tickBoundaryCases")
    void update_angleAroundTickBoundary_doesNotExceedOneStep(double requestedAngle,
                                                           double expectedAngle) {
        SteeringController steering = new SteeringController();

        double actualAngle = steering.update(requestedAngle, NORMAL_SPEED_KMH);

        assertEquals(new SteeringObservation(expectedAngle, expectedAngle, requestedAngle),
                new SteeringObservation(actualAngle, steering.getCurrentAngleDegrees(),
                        steering.getTargetAngleDegrees()));
    }

    static Stream<Arguments> tickBoundaryCases() {
        return Stream.of(
                arguments(Math.nextDown(60.0), Math.nextDown(60.0)),
                arguments(60.0, 60.0),
                arguments(Math.nextUp(60.0), 60.0),
                arguments(-Math.nextDown(60.0), -Math.nextDown(60.0)),
                arguments(-60.0, -60.0),
                arguments(-Math.nextUp(60.0), -60.0)
        );
    }

    @ParameterizedTest(name = "from {0} toward {1}")
    @CsvSource({"120.0, -450.0, 60.0", "-120.0, 450.0, -60.0",
            "-30.000000000000004, 30.0, 30.0", "30.000000000000004, -30.0, -30.0"})
    void update_oppositeRequestedDirection_movesOneStepTowardNewTarget(double initialAngle,
                                                                     double newTarget,
                                                                     double expectedAngle) {
        SteeringController steering = atAngle(initialAngle);

        double actualAngle = steering.update(newTarget, NORMAL_SPEED_KMH);

        assertEquals(new SteeringObservation(expectedAngle, expectedAngle, newTarget),
                new SteeringObservation(actualAngle, steering.getCurrentAngleDegrees(),
                        steering.getTargetAngleDegrees()));
    }

    @ParameterizedTest(name = "high-speed transition from direction {0}")
    @CsvSource({"1.0", "-1.0"})
    void update_enteringHighSpeed_reachesReducedTargetWithoutJumping(double direction) {
        SteeringController steering = atAngle(direction * PHYSICAL_LIMIT_DEGREES);

        double[] angles = new double[6];
        for (int tick = 0; tick < angles.length; tick++) {
            angles[tick] = steering.update(direction * PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_KMH);
        }

        double[] expected = Stream.of(390.0, 330.0, 270.0, 210.0, 150.0, 90.0)
                .mapToDouble(angle -> direction * angle).toArray();
        assertArrayEquals(expected, angles, TOLERANCE);

        assertEquals(new SteeringState(direction * HIGH_SPEED_LIMIT_DEGREES,
                        direction * HIGH_SPEED_LIMIT_DEGREES),
                new SteeringState(steering.getCurrentAngleDegrees(), steering.getTargetAngleDegrees()));
    }

    @Test
    void update_returningToNormalSpeed_restoresPhysicalTargetRange() {
        SteeringController steering = new SteeringController();
        advance(steering, PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_KMH, 2);

        double actualAngle = steering.update(PHYSICAL_LIMIT_DEGREES, NORMAL_SPEED_KMH);

        assertEquals(new SteeringObservation(150.0, 150.0, PHYSICAL_LIMIT_DEGREES),
                new SteeringObservation(actualAngle, steering.getCurrentAngleDegrees(),
                        steering.getTargetAngleDegrees()));
    }

    @ParameterizedTest(name = "invalid angle {0}")
    @MethodSource("autopilot.core.TestInputs#nonFiniteNumbers")
    void update_nonFiniteAngle_throwsIllegalArgument(double invalidAngle) {
        SteeringController steering = withPendingTarget();

        assertThrows(IllegalArgumentException.class,
                () -> steering.update(invalidAngle, NORMAL_SPEED_KMH));

        assertEquals(new SteeringState(MAX_STEP_DEGREES, PHYSICAL_LIMIT_DEGREES),
                new SteeringState(steering.getCurrentAngleDegrees(), steering.getTargetAngleDegrees()));
    }

    @ParameterizedTest(name = "invalid speed {0}")
    @MethodSource("autopilot.core.TestInputs#invalidNonNegativeNumbers")
    void update_invalidSpeed_throwsIllegalArgument(double invalidSpeed) {
        SteeringController steering = withPendingTarget();

        assertThrows(IllegalArgumentException.class,
                () -> steering.update(-PHYSICAL_LIMIT_DEGREES, invalidSpeed));

        assertEquals(new SteeringState(MAX_STEP_DEGREES, PHYSICAL_LIMIT_DEGREES),
                new SteeringState(steering.getCurrentAngleDegrees(), steering.getTargetAngleDegrees()));
    }

    @ParameterizedTest(name = "invalid angle {0} and invalid speed {1}")
    @MethodSource("multipleInvalidInputs")
    void update_multipleInvalidInputs_reportsAngleFailureFirst(double angle, double speed) {
        SteeringController steering = new SteeringController();

        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                () -> steering.update(angle, speed));

        assertEquals(ANGLE_VALIDATION_MESSAGE, failure.getMessage());
    }

    static Stream<Arguments> multipleInvalidInputs() {
        return Stream.of(arguments(Double.NaN, -1.0), arguments(Double.POSITIVE_INFINITY, Double.NaN),
                arguments(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY));
    }

    @ParameterizedTest(name = "self center from {0} gives {1}")
    @CsvSource(textBlock = """
              0.0,    0.0
             30.0,    0.0
            -30.0,    0.0
             30.5,    0.0
            -30.5,    0.0
             60.0,    0.0
            -60.0,    0.0
             61.0,    1.0
            -61.0,   -1.0
             60.5,    0.5
            -60.5,   -0.5
            120.0,   60.0
           -120.0,  -60.0
            450.0,  390.0
           -450.0, -390.0
            """)
    void selfCenter_currentAngle_movesTowardZeroWithoutOvershoot(double initialAngle,
                                                               double expectedAngle) {
        SteeringController steering = atAngle(initialAngle);

        double actualAngle = steering.selfCenter();

        assertEquals(new SteeringObservation(expectedAngle, expectedAngle, 0.0),
                new SteeringObservation(actualAngle, steering.getCurrentAngleDegrees(),
                        steering.getTargetAngleDegrees()));
    }

    @ParameterizedTest(name = "center repeatedly from direction {0}")
    @CsvSource({"1.0", "-1.0"})
    void selfCenter_repeatedCalls_reachesAndStaysAtZero(double direction) {
        SteeringController steering = atAngle(direction * PHYSICAL_LIMIT_DEGREES);
        double[] angles = new double[TICKS_TO_PHYSICAL_LIMIT + 1];

        for (int tick = 0; tick < angles.length; tick++) {
            angles[tick] = steering.selfCenter();
        }

        double[] expected = Stream.of(390.0, 330.0, 270.0, 210.0, 150.0, 90.0, 30.0, 0.0, 0.0)
                .mapToDouble(angle -> direction * angle).toArray();
        assertArrayEquals(expected, angles, TOLERANCE);

        assertEquals(new SteeringState(0.0, 0.0),
                new SteeringState(steering.getCurrentAngleDegrees(), steering.getTargetAngleDegrees()));
    }

    private static SteeringController withPendingTarget() {
        SteeringController steering = new SteeringController();
        steering.update(PHYSICAL_LIMIT_DEGREES, NORMAL_SPEED_KMH);
        return steering;
    }

    private static SteeringController atAngle(double angle) {
        SteeringController steering = new SteeringController();
        advance(steering, angle, NORMAL_SPEED_KMH, TICKS_TO_PHYSICAL_LIMIT);
        return steering;
    }

    private static double advance(SteeringController steering, double requestedAngle,
                                  double speedKmh, int ticks) {
        double angle = steering.getCurrentAngleDegrees();
        for (int tick = 0; tick < ticks; tick++) {
            angle = steering.update(requestedAngle, speedKmh);
        }
        return angle;
    }

    private record SteeringState(double angle, double target) {
    }

    private record SteeringObservation(double returnedAngle, double storedAngle, double target) {
    }
}
