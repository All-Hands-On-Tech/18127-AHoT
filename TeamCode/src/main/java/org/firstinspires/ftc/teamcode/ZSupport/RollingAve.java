package org.firstinspires.ftc.teamcode.ZSupport;

public class RollingAve {
    int size = 0;
    int counter = 0;
    double[] stack = new double[1000];

    public RollingAve(int numOfItems) {
        size = numOfItems;
    }

    public void addValue(double newValue) {
        stack[counter] = newValue;
        counter++;
        counter = counter % size;
    }

    public double getAverage() {
        double sum = 0;
        for (double value : stack) {
            sum += value;
        }

        return (sum/size);
    }
}
