package org.firstinspires.ftc.teamcode;

/**
 * doesnt work
 */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Gyro {
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

    private double angle = 0;
    private ElapsedTime timer;
    private double oldTime = 0;

    static final I2cAddr GYRO_ADDRESS = new I2cAddr(0x6B);
    private I2cDevice gyro;
    private I2cDeviceSynch gyroReader;

    private HardwareMap hwMap = null;

    final static double OFFSET = 18;
    final static double SCALE = 1 / 425;

    public Gyro(HardwareMap hardwareMap,
                                String gyroName//,
                                /*double milliSeconds,*/
                                /*int gain*/) {
        hwMap = hardwareMap;
        timer = new ElapsedTime();
        gyro = hardwareMap.i2cDevice.get(gyroName);
        gyroReader = new I2cDeviceSynchImpl(gyro, GYRO_ADDRESS, false);
        gyroReader.engage();
        gyroReader.write8(LOW_ODR, 0x00, true);  // Power on and enable ADC
        gyroReader.write8(CTRL4, 0x00, true); // Set gain
        gyroReader.write8(CTRL1, 0x6F, true);// Set integration time
    }

    public byte startPolling() {
        byte status = gyroReader.read8(STATUS);
        return status;
    }

    /*public byte getZH(byte status) {
        byte zDataH = 0;
        if(status >> 2 == 0x03){
            zDataH = gyroReader.read8(OUT_Z_H);
        }
        return zDataH;
    }

    public byte getZL(byte status) {
        byte zDataL = 0;
        if(status >> 2 == 0x03){
            zDataL = gyroReader.read8(0x2c);
        }
        return zDataL;
    }*/

    public void reset() {
        this.angle = 0;
    }

    public byte getZH() {
        return gyroReader.read8(OUT_Z_H);
    }

    public int getAngle() {
        int z = this.getZH();
        double diff = (z + Math.abs(z) / 18.) * (timer.milliseconds() - oldTime);
        angle += diff;
        oldTime = timer.milliseconds();
        return (int)Math.round(angle / SCALE);
    }
}
