package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

/**
 * Driver for ultrasonic distance sensor URM09.
 */

@I2cDeviceType
@DeviceProperties(name = "UltraSonic_URM09", description = "Ultrasonic distance sensor", xmlTag = "UltraSonicURM09")
public class urm09_customdriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x11);

    public urm09_customdriver(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

//    this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public double getDistanceCM() {
        byte[] regArray = deviceClient.read(0, 5);
        return regArray[4];
    }

    public double getDistanceCM300() {
        byte[] regArray = deviceClient.read(0, 4);
        return regArray[3];
    }

    public double getDistanceInch() {
        byte[] regArray = deviceClient.read(0, 5);
        double distanceDouble = regArray[4];
        return convertWrapAround_inch(distanceDouble);
    }

    public byte[] getAllRegister() {
        return deviceClient.read(0, 9);
    }

    //  protected void setMeasureRange(int addr, int range){
//    deviceClient.write8(addr, range);
//  }
    public void setMeasureRange(int range) {
        deviceClient.write8(7, range);
    }

    protected byte[] readByte(urm09_customdriver.Register reg) {
        return deviceClient.read(reg.bVal, 1);
    }

    public double convertWrapAround_inch(double distance) {
        double inchDistance = distance / 2.54;
        if (inchDistance < 0) {
            inchDistance = 50.0 + (50 + inchDistance);
        }
        return inchDistance;
    }

    @Override
    protected synchronized boolean doInitialize() {
//    int configSettings = Hysteresis.HYST_1_5.bVal | AlertControl.ALERT_ENABLE.bVal;
//    writeShort(Register.CONFIGURATION, (short) configSettings);
//     Mask out alert signal bit, which we can't control
//    return (readShort(Register.CONFIGURATION) & 0xFFEF) == configSettings;
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Gravity: URM09 Ultrasonic Sensor (IÂ²C)";
    }

    public enum DeviceParameter {
        //    MODE_PORT(0x08),
        PASSIVEMEASURE_MODE(0x01),
        AUTOMEASURE_MODE(0x00),
        RANGE_PORT(0x07),
        AUTOMEASURE_RANGE_150(0x80),    //1000 0000
        AUTOMEASURE_RANGE_300(0x90),    //1001 0000
        AUTOMEASURE_RANGE_500(0xA0);     //1010 0000

        public int bVal;

        DeviceParameter(int bVal) {
            this.bVal = bVal;
        }
    }

    public enum Register {
        ADDR(0x00),
        PRODUCT_ID(0x01),
        VERSION(0x02),
        DISTANCE_HIGH_BIT(0x03),    //CM
        DISTANCE_LOW_BIT(0x04),     //CM
        TEMP_HIGH_BIT(0x05),        //10x actual temperature
        TEMP_low_BIT(0x06),         //10x actual temperature
        CONFIGURE(0x07),            //passive_auto measurement, distance range set
        COMMAND(0x08);              //passive measurement control

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

}