package lib.BlueShift.constants;

public class CTRECANDevice {
    int deviceID;
    String canbus;

    public CTRECANDevice(int deviceID) {
        this(deviceID, "*");
    }

    public CTRECANDevice(int deviceID, String canbus) {
        this.deviceID = deviceID;
        this.canbus = canbus;
    }

    public int getDeviceID() {
        return deviceID;
    }

    public String getCanbus() {
        return canbus;
    }
}
