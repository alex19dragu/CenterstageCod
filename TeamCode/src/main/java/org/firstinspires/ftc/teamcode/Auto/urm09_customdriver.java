package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "URM09", xmlTag = "Urm09")
public class urm09_customdriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x11);

    public urm09_customdriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.DFRobot;
    }

    @Override
    protected synchronized boolean doInitialize() {
        try {
            // Automatic measurement mode (Bit7 = 1) and set the maximum ranging distance
            int configSettings = 0x80 | DistanceMode.MODE_300CM.bVal; // example: automatic measurement, 300CM
            deviceClient.write8(Register.CONFIGURE.bVal, configSettings);
            // Verify configuration
            return deviceClient.read8(Register.CONFIGURE.bVal) == configSettings;
        } catch (Exception e) {
            System.out.println("Error during initialization: " + e.getMessage());
            return false;
        }
    }

    @Override
    public String getDeviceName() {
        return "DFRobot URM09";
    }

    public enum Register {
        FIRST(0),
        ADDR(0x00),
        PRODUCT_ID(0x01),
        VERSION(0x02),
        DISTANCE_HIGH_BIT(0x03),    // CM
        DISTANCE_LOW_BIT(0x04),     // CM
        TEMP_HIGH_BIT(0x05),        // 10x actual temperature
        TEMP_LOW_BIT(0x06),         // 10x actual temperature
        CONFIGURE(0x07),            // passive_auto measurement, distance range set
        COMMAND(0x08),              // passive measurement control
        LAST(COMMAND.bVal);

        public int bVal;
        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    protected void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public enum DistanceMode {
        MODE_100CM(0x00),
        MODE_300CM(0x01);

        public int bVal;

        DistanceMode(int bVal) {
            this.bVal = bVal;
        }
    }

    public int getDistanceCM() {
        try {
            // Read 2 bytes starting from DISTANCE_HIGH_BIT
            byte highByte = deviceClient.read8(Register.DISTANCE_HIGH_BIT.bVal);
            byte lowByte = deviceClient.read8(Register.DISTANCE_LOW_BIT.bVal);
            int distance = ((highByte & 0xFF) << 8) | (lowByte & 0xFF);
            return distance;
        } catch (Exception e) {
            System.out.println("Error reading distance: " + e.getMessage());
            return -1; // Indicating an error
        }
    }
}
