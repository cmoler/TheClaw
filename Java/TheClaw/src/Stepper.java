public enum Stepper {
    BASE(0), UPPER_ARM(1), FORE_ARM(2), GRIPPER(3);

    private final int value;

    private Stepper(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
