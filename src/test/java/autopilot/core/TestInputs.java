package autopilot.core;

import java.util.stream.DoubleStream;

final class TestInputs {
    static final double TOLERANCE = 1.0e-9;

    private TestInputs() {
    }

    static DoubleStream invalidNonNegativeNumbers() {
        return DoubleStream.of(-Double.MIN_VALUE, -1.0, -Double.MAX_VALUE,
                Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }

    static DoubleStream nonFiniteNumbers() {
        return DoubleStream.of(Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }
}
