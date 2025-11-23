package org.firstinspires.ftc.teamcode.util;

public enum ShooterPower {
    OFF(0),
    LOW(0.25),
    MEDIUM(0.5),
    SEMI(0.75),
    HIGH(1.0);

    private final double power;

    ShooterPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }
}
