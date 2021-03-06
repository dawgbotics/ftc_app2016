/*
 * MIT License
 *
 * Copyright (c) 2016 Chris D
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by Chris D on 10/5/2016.
 *
 * Partially based on:
 * https://github.com/OliviliK/FTC_Library/blob/master/TCS34725_ColorSensor.java
 */
public class MultiplexColorSensor {

    // Registers
    static final int ENABLE = 0x80;
    static final int ATIME = 0x81;
    static final int CONTROL = 0x8F;
    static final int ID = 0x92;
    static final int STATUS = 0x93;
    static final int CDATAL = 0x94;

    // I2C address for color sensor
    static final I2cAddr ADA_ADDRESS = new I2cAddr(0x29);
    private I2cDevice ada;
    private I2cDeviceSynch adaReader;

    public static int GAIN_1X =  0x00;
    public static int GAIN_4X =  0x01;
    public static int GAIN_16X = 0x02;
    public static int GAIN_60X = 0x03;

    /**
     * Initializes Adafruit color sensors on the specified ports of the I2C
     * multiplexer.
     *
     * @param hardwareMap  hardwareMap from OpMode
     * @param colorName    Configuration name of I2CDevice for color sensor
     * @param milliSeconds Integration time in milliseconds
     * @param gain         Gain (GAIN_1X, GAIN_4X, GAIN_16X, GAIN_60X)
     */
    public MultiplexColorSensor(HardwareMap hardwareMap,
                                String colorName,
                                double milliSeconds,
                                int gain) {

        ada = hardwareMap.i2cDevice.get(colorName);
        adaReader = new I2cDeviceSynchImpl(ada, ADA_ADDRESS, false);
        adaReader.engage();

        final int time = integrationByte(milliSeconds);
        adaReader.write8(ENABLE, 0x03, true);  // Power on and enable ADC
        adaReader.read8(ID);                   // Read device ID
        adaReader.write8(CONTROL, gain, true); // Set gain
        adaReader.write8(ATIME, time, true);   // Set integration time
    }

    private int integrationByte(double milliSeconds) {
        int count = (int)(milliSeconds/2.4);
        if (count<1)    count = 1;   // Clamp the time range
        if (count>256)  count = 256;
        return (256 - count);
    }

    // Un-needed?
    public void startPolling() {
        adaReader.read8(STATUS);
    }

    /**
     * Retrieve the color read by the given color sensor
     *
     * @return Array containing the Clear, Red, Green, and Blue color values
     */
    public int[] getCRGB() {

        // Read color registers
        byte[] adaCache = adaReader.read(CDATAL, 8);

        // Combine high and low bytes
        int[] crgb = new int[4];
        for (int i=0; i<4; i++) {
            crgb[i] = (adaCache[2*i] & 0xFF) + (adaCache[2*i+1] & 0xFF) * 256;
        }
        return crgb;
    }
}