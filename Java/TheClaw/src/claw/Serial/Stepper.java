package claw.Serial;

public enum Stepper {
    BASE(0), LOWER_ARM(1), UPPER_ARM(2), GRIPPER(3);

    private final int value;

    private Stepper(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
