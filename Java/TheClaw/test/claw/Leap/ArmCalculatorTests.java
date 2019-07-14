package claw.Leap;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ArmCalculatorTests {

    /*
        Constants: Max & Min values for Length and Angles
     */
    static final int LOWER_ARM_LEN = 90;
    static final int UPPER_ARM_LEN = 50;
    static final int GRIPPER_LEN = 68;
    static final int TOTAL_ARM_LEN = LOWER_ARM_LEN + UPPER_ARM_LEN + GRIPPER_LEN; // 208

    // All angle calculations use radians, casted back to degrees before sent to arduino
    static final double LOWER_ARM_ANGLE_MIN = Math.toRadians(20); // 0.349
    static final double LOWER_ARM_ANGLE_MAX = Math.toRadians(90); // 1.571
    static final double UPPER_ARM_ANGLE_MIN = Math.toRadians(40); // 0.698
    static final double UPPER_ARM_ANGLE_MAX = Math.toRadians(180); // 3.142
    static final double BASE_ANGLE_MIN = Math.toRadians(15);
    static final double BASE_ANGLE_MAX = Math.toRadians(165); // 2.618

    static ArmCalculator armCalculator;

    @BeforeAll()
    public static void setup() {
        armCalculator = new ArmCalculator(
                LOWER_ARM_LEN,
                UPPER_ARM_LEN,
                GRIPPER_LEN
        );
        armCalculator.setBaseLimits(BASE_ANGLE_MIN, BASE_ANGLE_MAX);
        armCalculator.setLowerArmLimits(LOWER_ARM_ANGLE_MIN, LOWER_ARM_ANGLE_MAX);
        armCalculator.setUpperArmLimits(UPPER_ARM_ANGLE_MIN, UPPER_ARM_ANGLE_MAX);
    }

    @Test
    public void lerpFloat_minDist() {
        float start = -1.0f;
        float target = -0.1f;
        float lerpFactor = 0.3f;
        float minDist = 1.0f;

        float actualValue = armCalculator.lerpFloat(start, target, lerpFactor, minDist);

        assertEquals(actualValue, target);
    }

    @Test
    public void calcInverseKinetics_negative_y() {
        float xz = 100;
        float y = -10;

        double[] actualAngles = armCalculator.calcInverseKinematics(y, xz);

        assertEquals(2, actualAngles.length);
        assertEquals(true, Math.abs(actualAngles[0] - 1.232493) < 0.1);
        assertEquals(true, Math.abs(actualAngles[1] - 0.974727) < 0.1);
    }

    @Test
    public void calcInverseKinetics_hypot_short() {
        float xz = 30;
        float y = -10;

        double[] actualAngles = armCalculator.calcInverseKinematics(y, xz);

        assertEquals(2, actualAngles.length);
        assertEquals(true, Math.abs(actualAngles[0] - 1.443671) < 0.1);
        assertEquals(true, Math.abs(actualAngles[1] - 0.698232) < 0.1);
    }

    @Test
    public void calcInverseKinetics_hypot_long() {
        float xz = 2000;
        float y = 2000;

        double[] actualAngles = armCalculator.calcInverseKinematics(y, xz);

        assertEquals(2, actualAngles.length);
        assertEquals(true, Math.abs(actualAngles[0] - 0.785398) < 0.1);
        assertEquals(true, Math.abs(actualAngles[1] - 3.141593) < 0.1);
    }

    @Test
    public void setUpperArmLimits_innerHypot() {
        armCalculator.setUpperArmLimits(UPPER_ARM_ANGLE_MIN, UPPER_ARM_ANGLE_MAX);

        double actual = armCalculator.getInnerHypotMin();

        assertEquals((75.859 - actual) < 0.1, true);
    }
}