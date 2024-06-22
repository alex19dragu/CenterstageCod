package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

@TeleOp(name = "I2C Address Scanner", group = "Sensor")
public class checkadress extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Scanning for I2C devices...");
        telemetry.update();

        // Loop through all possible I2C addresses
        for (int i = 0x08; i <= 0x77; i++) {
            I2cAddr address = I2cAddr.create7bit(i);

            try {
                // Create a temporary I2cDeviceSynchSimple object
                I2cDeviceSynchSimple device = hardwareMap.get(I2cDeviceSynchSimple.class, "ultrasonicSensor");
                device.setI2cAddress(address);

                // Try reading a byte from the device to see if it responds
                device.read(0, 1);
                telemetry.addData("Device found at address", "0x%02X", i);
                telemetry.update();
            } catch (Exception e) {
                // Do nothing, the address is not responding
            }
        }

        telemetry.addData("Status", "Scan complete");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Keep the OpMode running
        while (opModeIsActive()) {
            idle();
        }
    }
}
