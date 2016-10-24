package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Gyro {
    //I2C device addresses
    final static int CTRL1 = 0x20;
    final static int CTRL4 = 0x23;
    final static int STATUS = 0xA7;

    final static int OUT_X_L = 0xA8;
    final static int OUT_X_H = 0xA9;
    final static int OUT_Y_L = 0xAA;
    final static int OUT_Y_H = 0xAB;
    final static int OUT_Z_L = 0xAC;
    final static int OUT_Z_H = 0xAD;

    final static int LOW_ODR = 0x39;

    //instance variables
    private double angle = 0;
    private ElapsedTime timer;
    private double oldTime = 0;
    final static double OFFSET = 18.;
    final static double SCALE = 1 / 425;

    static final I2cAddr GYRO_ADDRESS = new I2cAddr(0x6B);
    private I2cDevice gyro;
    private I2cDeviceSynch gyroReader;

    /**
     * creates gyro object and initializes the I2C device and the
     * I2C device reader.
     *
     * @param hardwareMap hardware map of robot
     * @param gyroName name of gyro within hardware map
     */
    public Gyro(HardwareMap hardwareMap, String gyroName) {
        timer = new ElapsedTime();
        gyro = hardwareMap.i2cDevice.get(gyroName);
        gyroReader = new I2cDeviceSynchImpl(gyro, GYRO_ADDRESS, false);
        gyroReader.engage();
        gyroReader.write8(LOW_ODR, 0x00, true);  // Power on and enable ADC
        gyroReader.write8(CTRL4, 0x00, true); // Set gain
        gyroReader.write8(CTRL1, 0x6F, true);// Set integration time
    }

    /**
     * Starts gyro polling by reading the status from it
     *
     * @return returns the current status of the gyro
     */
    public byte startPolling() {
        return gyroReader.read8(STATUS);
    }

    /**
     * resets gyro, setting angle back to zero
     */
    public void reset() {
        this.angle = 0;
    }

    /**
     * gives current angular velocity across z axis
     *
     * @return z angular velocity
     */
    public byte getZ() {
        return gyroReader.read8(OUT_Z_H);
    }

    /**
     * Reads data from gyro, setting angle accordingly
     */
    public void read() {
        int z = this.getZ();
        double diff = (z + Math.abs(z) / OFFSET) * (timer.milliseconds() - oldTime);
        angle += diff;
        oldTime = timer.milliseconds();
    }

    /**
     * returns the current angle
     *
     * @return current angle
     */
    public int getAngle() {
        return (int)Math.round(angle / SCALE);
    }

    /**
     * closes both I2C devices
     */
    public void close() {
        gyro.close();
        gyroReader.close();
    }
}