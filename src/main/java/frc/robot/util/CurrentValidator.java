package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CurrentValidator implements BooleanSupplier {

    private final DoubleSupplier getCurrent;
    private final double limit;
    private final int cutoff;
    private double current = 0.0;

    /**
     * A limit to detect whether a motor's current draw is below a threshold with a
     * low-pass filter
     * 
     * @param limit      The current limit in amps
     * @param getCurrent A DoubleSupplier that supplies a motor's current draw
     * @param cutoff     The cutoff for the filter
     */
    public CurrentValidator(double limit, DoubleSupplier getCurrent, int cutoff) {
        this.limit = limit;
        this.getCurrent = getCurrent;
        this.cutoff = cutoff;
    }

    public boolean getAsBoolean() {
        current = (current * (cutoff - 1) + getCurrent.getAsDouble()) / ((double) cutoff);
        return current < limit;
    }
}