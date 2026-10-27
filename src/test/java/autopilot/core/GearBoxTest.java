package autopilot.core;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;
import autopilot.sensor.SpeedSensor;

class GearBoxTest {

    private GearBox gearBox;
    private double currentSpeed;

    @BeforeEach
    void setUp() {
        SpeedSensor speedSensor = () -> currentSpeed;
        gearBox = new GearBox(speedSensor);
    }

    @Test
    void testNegativeSpeedThrowsException() {
        currentSpeed = -1.0;
        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(GearBox.Gear.R));
    }

    @Test
    void testNullTargetGearThrowsException() {
        assertThrows(NullPointerException.class, () -> gearBox.changeTo(null));
    }

    @Test
    void testChangeFromReverseToParkThrowsException() {
        currentSpeed = 0.0;
        gearBox.changeTo(GearBox.Gear.R);
        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.P));
    }

    @ParameterizedTest
    @ValueSource(doubles = {0.0, 1.0, 3.0, 4.0})
    void testValidChangeFromParkToReverse(double speed) {
        currentSpeed = speed;
        assertDoesNotThrow(() -> gearBox.changeTo(GearBox.Gear.R));
        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    @ParameterizedTest
    @ValueSource(doubles = {5.0, 6.0, 7.0, 8.0})
    void testInvalidChangeFromParkToReverse(double speed) {
        currentSpeed = speed;
        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.R));
    }

    @ParameterizedTest
    @CsvSource({
        "0.0, true",
        "1.0, false",
        "2.0, false"
    })
    void testChangeBetweenReverseAndDrive(double speed, boolean isAllowed) {
        currentSpeed = 0.0;
        gearBox.changeTo(GearBox.Gear.R);
        
        currentSpeed = speed;
        if (isAllowed) {
            assertDoesNotThrow(() -> gearBox.changeTo(GearBox.Gear.D));
            assertDoesNotThrow(() -> gearBox.changeTo(GearBox.Gear.R));
        } else {
            assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.D));
        }
    }

    @ParameterizedTest
    @ValueSource(doubles = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0})
    void testValidChangeFromReverseToNeutral(double speed) {
        currentSpeed = 0.0;
        gearBox.changeTo(GearBox.Gear.R);
        
        currentSpeed = speed;
        assertDoesNotThrow(() -> gearBox.changeTo(GearBox.Gear.N));
    }

    @ParameterizedTest
    @ValueSource(doubles = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0})
    void testValidChangeFromNeutralToDrive(double speed) {
        currentSpeed = 0.0;
        gearBox.changeTo(GearBox.Gear.R);
        gearBox.changeTo(GearBox.Gear.N);
        
        currentSpeed = speed;
        assertDoesNotThrow(() -> gearBox.changeTo(GearBox.Gear.D));
    }

    @Test
    void testInvalidChangesNeutralToParkAndDriveToNeutral() {
        currentSpeed = 0.0;
        gearBox.changeTo(GearBox.Gear.R);
        gearBox.changeTo(GearBox.Gear.N);
        
        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.P));
        
        gearBox.changeTo(GearBox.Gear.D);
        
        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.N));
    }


    @Test
    void testSensorFailurePropagates() {
        SensorFailureException expectedFailure = new SensorFailureException("Speed reading unavailable");
        gearBox = new GearBox(() -> {
            throw expectedFailure;
        });

        SensorFailureException actualFailure = assertThrows(SensorFailureException.class, () -> gearBox.changeTo(GearBox.Gear.R));
        assertEquals(expectedFailure, actualFailure);
    }

    @Test
    void testChangeToSameGearDoesNotReadSensor() {
        gearBox = new GearBox(() -> {
            throw new AssertionError("An unchanged gear must not read the speed sensor");
        });

        assertDoesNotThrow(() -> gearBox.changeTo(GearBox.Gear.P));
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void testRejectedTransitionPreservesCurrentGear() {
        currentSpeed = 6.0;
        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.R));
        
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void testInvalidSpeedDoesNotChangeGearBeforeRecovery() {
        currentSpeed = -1.0; 
        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(GearBox.Gear.R));
        
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }
}