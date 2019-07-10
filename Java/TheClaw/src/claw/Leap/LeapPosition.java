package claw.Leap;

/*
    Object that holds angles from claw.Leap Motion
 */
public class LeapPosition {

    // Float degree values for arduino usage
    public float baseAngle;
    public float lowerArmAngle;
    public float upperArmAngle;
    public float gripperAngle = 180;
    public float gripperRatio;

    public LeapPosition() {
    }

    public void updateAngles(float baseAngle, float lowerArmAngle, float upperArmAngle, float gripperRatio) {
        this.baseAngle = baseAngle;
        this.lowerArmAngle = lowerArmAngle;
        this.upperArmAngle = upperArmAngle;
        this.gripperRatio = gripperRatio;
    }

    @Override()
    public String toString() {
        return String.format(
                "Base: %10f; Lower: %10f; Upper: %10f; Gripper: %10f",
                this.baseAngle,
                this.lowerArmAngle,
                this.upperArmAngle,
                this.gripperRatio
        );
    }
}
