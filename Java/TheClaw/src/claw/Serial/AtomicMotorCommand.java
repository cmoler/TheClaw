package claw.Serial;

public class AtomicMotorCommand {

    public MotorCommand[] commands;
    public long ttlMs;
    private long startTimeMs;

    public AtomicMotorCommand(long ttlMs, MotorCommand ...commands) {
        this.ttlMs = ttlMs;
        this.commands = commands;
        this.startTimeMs = System.currentTimeMillis();
    }

    public boolean isFresh() {
        return (System.currentTimeMillis() - this.startTimeMs) <= ttlMs;
    }
}
