/*
Running Average Class, from ChatGPT
Uses a fixed-size queue (circular buffer) to store the last n numbers. 
When a new number is added and the queue is full, the oldest number is removed. 
This way, you always maintain the last n numbers for your average calculation.
*/

package org.firstinspires.ftc.teamcode;

import java.util.LinkedList;
import java.util.Queue;


public class RunningAverage {
    private final int maxSize;
    private final Queue<Double> window;
    private double sum;

    // Constructor
    public RunningAverage(int size) {
        this.maxSize = size;
        this.window = new LinkedList<>();
        this.sum = 0.0;
    }

    public void addNumber(double number) {
        if (window.size() == maxSize) {
            sum -= window.poll(); // Remove the oldest number from the sum
        }
        window.add(number);
        sum += number;
    }

    public double getAverage() {
        if (window.isEmpty()) {
            return 0; // Or throw an exception if no numbers have been added yet
        }
        return sum / window.size();
    }

    /*  Example usage
    public static void main(String[] args) {
        int n = 3; // The size of the window
        RunningAverage runningAverage = new RunningAverage(n);
        
        double[] numbers = {10, 20, 30, 40, 50};
        for (double number : numbers) {
            runningAverage.addNumber(number);
            System.out.println("Added: " + number + ", 
             Running Average of last " + n + " numbers: " + runningAverage.getAverage());
        }
    }
    */
}
