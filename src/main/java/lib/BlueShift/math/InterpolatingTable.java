package lib.BlueShift.math;

import java.util.Arrays;

/**
 * linearInterpolator - given a table of x and y values, will interpolate values
 * of y between known values of x using linear interpolation.
 * 
 * Usage: private double[][] data = { {1.0, 10.0}, {3.0, 31.0}, {10,100} };
 * private linearInterpolator lt = new linearInterpolator(data);
 * 
 * double y = lt.getInterpolatedValue(1.5); // returns 15.25
 */
public class InterpolatingTable {

    private double[][] table;
    private boolean initialized = false;

    public InterpolatingTable(double[][] data) {
        build_table(data);
    }

    public boolean isInitialized() {
        return initialized;
    }

    private void build_table(double[][] data) {
        int rows = data.length;
        if (rows < 1) {
            System.out.println("ERROR: linearInterpolator needs at least one data point.");
            return;
        }
        int cols = data[0].length;
        if (cols != 2) {
            System.out.println("ERROR: linearInterpolator number of columns should be 2");
            return;
        }

        table = new double[rows][cols];
        for (int x = 0; x < data.length; x++) {
            for (int y = 0; y < data[x].length; y++) {
                table[x][y] = data[x][y];
            }
        }
        Arrays.sort(table, (a, b) -> Double.compare(a[0], b[0]));
        initialized = true;
    }

    /**
     * Binary search implementation to find the closest two x values in the table
     * for interpolation.
     */
    private int binarySearch(double x) {
        int low = 0;
        int high = table.length - 1;

        while (low <= high) {
            int mid = (low + high) / 2;

            if (table[mid][0] == x) {
                return mid;  // Exact match
            } else if (table[mid][0] < x) {
                low = mid + 1;
            } else {
                high = mid - 1;
            }
        }

        // Return the index where the x value would be inserted, to determine the
        // interpolation range.
        return low;
    }

    public double getInterpolatedValue(double x) {
        if (!initialized) {
            System.out.println("ERROR: linearInterpolator number of columns should be 2");
            return 0.0;
        }

        // Use binary search to find the closest x value
        int index = binarySearch(x);

        if (index >= table.length) {
            return table[table.length - 1][1];  // x is larger than the largest x in the table
        }

        if (index == 0) {
            return table[0][1];  // x is smaller than the smallest x in the table
        }

        double high_y = table[index][1];
        double high_x = table[index][0];
        if (high_x == x) {
            return high_y;  // Exact match found
        }

        double low_y = table[index - 1][1];
        double low_x = table[index - 1][0];

        // Linear interpolation formula
        return (low_y + (x - low_x) * (high_y - low_y) / (high_x - low_x));
    }
}
