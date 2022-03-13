package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;

public class ColorMath {
    public static Color add(Color a, Color b) {
        return new Color(a.red + b.red, a.green + b.green, a.blue + b.blue);
    }

    public static Color sub(Color a, Color b) {
        return new Color(a.red - b.red, a.green - b.green, a.blue - b.blue);
    }

    public static Color mul(Color a, double b) {
        return new Color(a.red * b, a.green * b, a.blue * b);
    }

    public static Color lerp(Color a, Color b, double fac) {
        // mx+b
        // (b - a) * fac + a

        return add(mul(sub(b, a), fac), a);
    }
}
