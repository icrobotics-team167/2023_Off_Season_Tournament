package frc.robot.util;

import java.util.LinkedList;
import java.util.Queue;

/**
 * A moving average filter, to smooth out noisy inputs such as current draw or
 * robot pitch.
 */
public class MovingAverage {

    private final int size;
    private final boolean weighted;
    private Queue<Double> inputs;

    /**
     * Constructs a moving average filter.
     * 
     * @param size How many ticks to store inputs for. 1 tick = 1/50th of a
     *             second.
     */
    public MovingAverage(int size) {
        this(size, false);
    }

    /**
     * Constructs a moving average filter.
     * 
     * @param size     How many ticks to store inputs for. 1 tick = 1/50th of a
     *                 second.
     * @param weighted Whether or not to weight recent data more.
     */
    public MovingAverage(int size, boolean weighted) {
        this.size = size;
        this.weighted = weighted;
        inputs = new LinkedList<>();
    }

    public void add(double input) {
        if (inputs.size() >= size) {
            inputs.remove();
        }
        inputs.add(input);
    }

    public double get() {
        double sum = 0;
        int count = 0;
        for (int i = 1; i <= inputs.size(); i++) {
            double x = inputs.remove();
            sum += weighted ? i * x : x;
            count += weighted ? i : 1;
            inputs.add(x);
        }
        return sum / count;
    }

    public void clear() {
        inputs.clear();
    }
}
