package lib.BlueShift.utils;

public class JoystickUtils {
    public static double applyDeadbband(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0;
        return value;
    }
}
