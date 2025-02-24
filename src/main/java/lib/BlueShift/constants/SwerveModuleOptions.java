package lib.BlueShift.constants;

public class SwerveModuleOptions {
    public int driveMotorID;
    public int turningMotorID;

    public String name;
    
    public CTRECANDevice absoluteEncoderDevice;

    public SwerveModuleOptions() {}

    public SwerveModuleOptions setDriveMotorID(int driveMotorID) {
        this.driveMotorID = driveMotorID;
        return this;
    }

    public SwerveModuleOptions setTurningMotorID(int turningMotorID) {
        this.turningMotorID = turningMotorID;
        return this;
    }

    public SwerveModuleOptions setName(String name) {
        this.name = name;
        return this;
    }

    public SwerveModuleOptions setAbsoluteEncoderCANDevice(CTRECANDevice absoluteEncoderDevice) {
        this.absoluteEncoderDevice = absoluteEncoderDevice;
        return this;
    }
}