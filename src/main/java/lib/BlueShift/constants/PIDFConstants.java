package lib.BlueShift.constants;

import edu.wpi.first.math.controller.PIDController;

public class PIDFConstants {
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    public PIDFConstants(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public PIDFConstants(double kP, double kI, double kD) {
        this(kP, kI, kD, 0);
    }

    public PIDFConstants(double kP, double kI) {
        this(kP, kI, 0);
    }

    public PIDFConstants(double kP) {
        this(kP, 0);
    }

    public PIDFConstants setP(double kP) {
        this.kP = kP;
        return this;
    }

    public PIDFConstants setI(double kI) {
        this.kI = kI;
        return this;
    }

    public PIDFConstants setD(double kD) {
        this.kD = kD;
        return this;
    }

    public PIDFConstants setF(double kF) {
        this.kF = kF;
        return this;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

    public PIDController toPIDController() {
        return new PIDController(kP, kI, kD);
    }
}
