package claw.Leap;

import com.leapmotion.leap.Vector;

public class ArmCalculator {

    private double baseAngleMax;
    private double baseAngleMin;
    private double lowerArmMin;
    private double lowerArmMax;
    private double upperArmMin;
    private double upperArmMax;
    private float lowerArmLen;
    private float upperArmLen;
    private float gripperLen;
    private double innerHypotMin;

    public ArmCalculator(float lowerArmLen, float upperArmLen, float gripperLen) {
        this.lowerArmLen = lowerArmLen;
        this.upperArmLen = upperArmLen;
        this.gripperLen = gripperLen;
    }

    public void setBaseLimits(double baseAngleMin, double baseAngleMax) {
        this.baseAngleMin = baseAngleMin;
        this.baseAngleMax = baseAngleMax;
    }

    public void setLowerArmLimits(double lowerArmMin, double lowerArmMax) {
        this.lowerArmMin = lowerArmMin;
        this.lowerArmMax = lowerArmMax;
    }

    public void setUpperArmLimits(double upperArmMin, double upperArmMax) {
        this.upperArmMin = upperArmMin;
        this.upperArmMax = upperArmMax;
        this.innerHypotMin = Math.sqrt(
                Math.pow(lowerArmLen, 2) + Math.pow(upperArmLen + gripperLen, 2)
                - (2 * lowerArmLen * (upperArmLen + gripperLen) * Math.cos(upperArmMin))
        );
    }

    public double getInnerHypotMin() {
        return this.innerHypotMin;
    }

    public double calcBaseAngle(float x, float z) {
        double angle = Math.atan2(z, x);

        // Restrict angles to min and max
        if (angle > baseAngleMax) {
            angle = baseAngleMax;
        }
        else if (angle < baseAngleMin) {
            angle = baseAngleMin;
        }
        return angle;
    }

    public double[] calcInverseKinematics(double y, double xz) {
        double h = Math.hypot(y, xz);
        double a1 = Math.atan2(y, xz);

        // Restrict hypotenuse to min and max robot arm range
        if (h > lowerArmLen + upperArmLen + gripperLen) {
            h = lowerArmLen + upperArmLen + gripperLen;
        } else if (h < innerHypotMin) {
            h = innerHypotMin;
            xz = Math.sqrt(Math.pow(h, 2) - Math.pow(y, 2));
            a1 = Math.atan2(y, xz);
        }

        // recalculate xz based on new h, fixed y


        // Lower arm angle is comprised of a1 (lower part using right triangle) and a2 (upper part using law of cosines)


        double a2 = Math.acos(
                (Math.pow(h, 2) + Math.pow(lowerArmLen, 2) - Math.pow(upperArmLen + gripperLen, 2))
                        / (2 * h * lowerArmLen));

        double lowerArmAngle = a1 + Math.abs(a2);
        lowerArmAngle = Math.min(Math.max(lowerArmAngle, lowerArmMin), lowerArmMax);

        // Upper arm angle calc using law of cosines
        double upperArmAngle = Math.acos(
                (Math.pow(lowerArmLen, 2) + Math.pow(upperArmLen + gripperLen, 2) - Math.pow(h, 2))
                    / (2 * lowerArmLen * (upperArmLen + gripperLen))
        );
        upperArmAngle = Math.min(Math.max(upperArmAngle, upperArmMin), upperArmMax);

        double[] angles = { lowerArmAngle, upperArmAngle };

        // logger.log(Level.DEBUG, String.format("%f,%f,%f,%f,%f,%f,%f", xz, y, h, a1, a2, lowerArmAngle, upperArmAngle));

        return angles;
    }

    public Vector lerpVector(Vector vecStart, Vector vecEnd, float lerpFactor, float minDist) {
        Vector vecOut;
        Vector diff = vecEnd.minus(vecStart);
        float dist = diff.magnitude();
        if (dist > minDist) {
            vecOut = vecStart.plus(diff.normalized().times(dist * lerpFactor));
        } else {
            vecOut = vecEnd;
        }
        return vecOut;
    }

    public float lerpFloat(float valStart, float valEnd, float lerpFactor, float minDist) {
        float out;
        float diff = valEnd - valStart;
        if (diff > minDist) {
            out = valStart + diff * lerpFactor;
        } else {
            out = valEnd;
        }
        return out;
    }
}
