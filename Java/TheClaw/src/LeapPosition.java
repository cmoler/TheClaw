/*
    Object that holds angles from Leap Motion
 */
public class LeapPosition {

    // Float degree values for arduino usage
    public float baseAngle;
    public float lowerArmAngle;
    public float upperArmAngle;
    public float gripperAngle = 180;

    public LeapPosition() {
    }

    public void updateAngles(float baseAngle, float lowerArmAngle, float upperArmAngle) {
        this.baseAngle = baseAngle;
        this.lowerArmAngle = lowerArmAngle;
        this.upperArmAngle = upperArmAngle;
    }
}
