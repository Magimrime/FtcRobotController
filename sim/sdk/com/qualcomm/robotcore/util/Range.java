package com.qualcomm.robotcore.util;

/** SIMULATOR copy of the FTC SDK utility (same behavior). */
public class Range {
    public static double scale(double n, double x1, double x2, double y1, double y2) {
        double a = (y1 - y2) / (x1 - x2);
        double b = y1 - x1 * (y1 - y2) / (x1 - x2);
        return a * n + b;
    }

    public static double clip(double number, double min, double max) {
        if (number < min) return min;
        if (number > max) return max;
        return number;
    }

    public static float clip(float number, float min, float max) {
        if (number < min) return min;
        if (number > max) return max;
        return number;
    }

    public static int clip(int number, int min, int max) {
        if (number < min) return min;
        if (number > max) return max;
        return number;
    }

    public static short clip(short number, short min, short max) {
        if (number < min) return min;
        if (number > max) return max;
        return number;
    }

    public static byte clip(byte number, byte min, byte max) {
        if (number < min) return min;
        if (number > max) return max;
        return number;
    }

    public static void throwIfRangeIsInvalid(double number, double min, double max) {
        if (number < min || number > max) {
            throw new IllegalArgumentException(String.format("number %f is invalid; valid ranges are %f..%f", number, min, max));
        }
    }

    public static void throwIfRangeIsInvalid(int number, int min, int max) {
        if (number < min || number > max) {
            throw new IllegalArgumentException(String.format("number %d is invalid; valid ranges are %d..%d", number, min, max));
        }
    }
}
