import com.leapmotion.leap.*;

/*
    Listener for the Leap in order to grab values from it once it is connected
 */
public class LeapListener extends Listener {

    /*
        Constants/ Max & Min values
     */
    static final int LOWER_ARM_LEN = 90;
    static final int UPPER_ARM_LEN = 50;
    static final int GRIPPER_LEN = 68;
    static final int TOTAL_ARM_LEN = LOWER_ARM_LEN + UPPER_ARM_LEN + GRIPPER_LEN;

    static final float LOWER_ARM_ANGLE_MIN = 20;
    static final float LOWER_ARM_ANGLE_MAX = 90;
    static final float UPPER_ARM_ANGLE_MIN = 40;
    static final float UPPER_ARM_ANGLE_MAX = 180;
    static final float BASE_ANGLE_MIN = 0;
    static final float BASE_ANGLE_MAX = 150;

    static final float Y_MAX = TOTAL_ARM_LEN;
    static final float X_MAX = (float) (Math.cos(LOWER_ARM_ANGLE_MIN) * TOTAL_ARM_LEN);

    // Inner hypotenuse angle calculated using law of cosines
    static final double INNER_HYPOT_MIN = (LOWER_ARM_LEN * LOWER_ARM_LEN)
            + ((UPPER_ARM_LEN + GRIPPER_LEN) * (UPPER_ARM_LEN + GRIPPER_LEN))
          - (2 * LOWER_ARM_LEN * (UPPER_ARM_LEN + GRIPPER_LEN) * Math.cos(UPPER_ARM_ANGLE_MIN));

    LeapPosition leapPosition;


    public void onInit(Controller controller) {
        System.out.println("Listener Initialized");
    }

    public void onConnect(Controller controller) {
        System.out.println("Connected");
        leapPosition = new LeapPosition();
    }

    public void onDisconnect(Controller controller) {
        System.out.println("Disconnected");
    }

    public void onExit(Controller controller) {
        System.out.println("Exited");
    }

    public void onFrame(Controller controller) {

        System.out.println("Frame available");

        Frame frame = controller.frame();

        if (!frame.hands().isEmpty()) {
            Vector palmPosition = frame.hands().get(0).palmPosition();

            // Restrict Leap Space
            //TODO: normalize correctly?
            if (palmPosition.getY() > Y_MAX) {
                palmPosition.setY(Y_MAX);
            }

            if (palmPosition.getX() > X_MAX) {
                palmPosition.setX(X_MAX);
            }

            if (palmPosition.getZ() > X_MAX) {
                palmPosition.setZ(X_MAX);
            }

            // Calculate Angles
            float baseAngle = calcBaseAngle(palmPosition.getX(), palmPosition.getZ());
            float xz = (float) Math.hypot(palmPosition.getX(), palmPosition.getZ());
            float[] kinematicsAngles = calcInverseKinematics(palmPosition.getY(), xz);

            // Set all angles within the LeapPosition
            leapPosition.updateAngles(baseAngle, kinematicsAngles[0], kinematicsAngles[1]);
        }
    }

    private float calcBaseAngle(float x, float z) {
        float angle = (float) Math.toDegrees(Math.tan(z / x));

        // Restrict angles to min and max
        if (angle > BASE_ANGLE_MAX) {
            angle = BASE_ANGLE_MAX;
        }
        else if (angle < BASE_ANGLE_MIN) {
            angle = BASE_ANGLE_MIN;
        }
        return angle;
    }

    private float[] calcInverseKinematics(float y, float xz) {
      double h = Math.hypot(y, xz);

      // Restrict angles to min and max
      if (h > TOTAL_ARM_LEN) {
        h = TOTAL_ARM_LEN;
      } else if (h < INNER_HYPOT_MIN) {
        h = INNER_HYPOT_MIN;
      }

      // Lower arm angle is comprised of a1 (lower part using right triangle) and a2 (upper part using law of cosines)
      double a1 = Math.atan(y/xz);

      double a2 = Math.acos(
              ((h * h) + (LOWER_ARM_LEN * LOWER_ARM_LEN)
                      - ((UPPER_ARM_LEN + GRIPPER_LEN) * (UPPER_ARM_LEN + GRIPPER_LEN)))
                      / (2 * h * LOWER_ARM_LEN));

      // Upper arm angle calc using law of cosines
      double upperArmAngle = Math.acos(
              ((LOWER_ARM_LEN * LOWER_ARM_LEN)
                      + ((UPPER_ARM_LEN + GRIPPER_LEN) * (UPPER_ARM_LEN + GRIPPER_LEN))
                      - (h * h))
                      / (2 * LOWER_ARM_LEN * (UPPER_ARM_LEN + GRIPPER_LEN)));

      float[] angles = {(float) Math.toDegrees(a1 + a2), (float) Math.toDegrees(upperArmAngle)};

      return angles;
    }
}