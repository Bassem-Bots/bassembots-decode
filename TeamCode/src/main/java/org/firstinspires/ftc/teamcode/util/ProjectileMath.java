package org.firstinspires.ftc.teamcode;

public class ProjectileMath {

    public static class ShotResult {
        public final double angleRadians;
        public final double velocity;

        public ShotResult(double angleRadians, double velocity) {
            this.angleRadians = angleRadians;
            this.velocity = velocity;
        }
    }

    /**
     * Calculates launch angle (radians) and velocity for a projectile.
     * @param q horizontal distance to target
     * @param w target height
     * @param d maximum arc height
     * @param g gravitational acceleration (9.8 on Earth)
     */
    public static ShotResult computeShot(double q, double w, double d, double g) {
        double z = Math.sqrt(2 * g * d);                     // vertical velocity component
        double b = Math.sqrt((2 * g * d) - (2 * g * w)) + z; // adjusted horizontal component
        double n = Math.atan((z * b) / (q * g));             // launch angle
        double v = Math.sqrt(Math.pow((q * g / b), 2) + (2 * g * d)); // total velocity

        return new ShotResult(n, v);
    }
}
