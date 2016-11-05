package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

public class Drive {
    double xComp;
    double yComp;
    double rot;
    int oldGyro = OFFSET;

    public static final double BASE_SPEED = .5;
    public static final int OFFSET = 45;

    static final int ROT_RATIO = 100;

    DcMotor motorLeftFront;
    DcMotor motorRightFront;
    DcMotor motorLeftBack;
    DcMotor motorRightBack;

    Gyro gyro;

    public Drive(HardwareMap hardwareMap, String gyroName) {
        motorLeftBack = hardwareMap.dcMotor.get("left back");
        motorLeftFront = hardwareMap.dcMotor.get("left front");
        motorRightBack = hardwareMap.dcMotor.get("right back");
        motorRightFront = hardwareMap.dcMotor.get("right front");

        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro = new Gyro(hardwareMap, gyroName);
    }

    public boolean driveToPosition(int targetTicks, double speed) {
        int currentTicks = (int)max(motorLeftBack.getCurrentPosition(), motorLeftFront.getCurrentPosition(),
               motorRightBack.getCurrentPosition(), motorRightFront.getCurrentPosition());
        if (currentTicks <= targetTicks) {
            speed *= BASE_SPEED + ((targetTicks - currentTicks) / targetTicks) * (1 - BASE_SPEED);
        } else {
            drive(0);
            return false;
        }
        drive(speed);
        return true;
    }

    public void useGyro() {
        gyro.readZ();
        double r;
        if (rot == 0) {
            double gyroDiff = gyro.getAngleZ() - oldGyro;
            //If you're moving forwards and you drift, this should correct it.
            //Accounts for if you go from 1 degree to 360 degrees which is only a difference of one degree,
            //but the bot thinks that's 359 degree difference
            //Scales -180 to 180 -> -1 to 1
            if (gyroDiff < -180) {
                r = (180 + gyroDiff) / 180; //replaced (1.5 * (gyroDiff/180)) because function of 1.5 is unknown
            }
            if (gyroDiff > 180) {
                r = (180 - gyroDiff) / 180; //replaced (1.5 * (gyroDiff/180)) because function of 1.5 is unknown
            } else {
                r = (gyroDiff - 180) / 180; //replaced (1.5 * (gyroDiff/180)) because function of 1.5 is unknown
            }
        } else {
            oldGyro = gyro.getAngleZ();
            r = rot;
        }

        rot = Range.clip(r, -1, 1);

        double temp = xComp;
        xComp = xComp * Math.cos(gyro.getAngleZ()) - yComp * Math.sin(gyro.getAngleZ());
        yComp = temp * Math.sin(gyro.getAngleZ()) + yComp * Math.cos(gyro.getAngleZ());
    }

    public void drive(double speed) {
        double[] speedWheel = new double[4];

        for (int n = oldGyro; n < 360; n += 90) {
            //This \/ rotates the control input to make it work on each motor
            speedWheel[n] = (xComp * Math.sin(n) + yComp * Math.cos(n) + ROT_RATIO * rot);
        }

        /*
        Indexes:
        0 is the right front motor
        1 is the left front motor
        2 is the left back motor
        3 is the right back motor
         */

        //In order to handle the problem if the values in speedWheel[] are greater than 1,
        //this scales them so the ratio between the values stays the same, but makes sure they're
        //less than 1
        double scaler = Math.abs(1 / max(speedWheel[0], speedWheel[1], speedWheel[2], speedWheel[3]));

        for (int n = 0; n < 4; n++) {
            speedWheel[n] = speed * scaler * speedWheel[n];
        }

        motorRightFront.setPower(speedWheel[0]);
        motorLeftFront.setPower(speedWheel[1]);
        motorLeftBack.setPower(speedWheel[2]);
        motorRightBack.setPower(speedWheel[3]);
    }

    public double max(double a, double b, double c, double d) {
        double max = a;
        double[] vals = {b, c, d};

        for (int i = 0; i < 3; i++) {
            if (vals[i] > max) {
                max = vals[i];
            }
        }
        return max;

        //We can't use a long loop because it has to take less time than a phone tick, so if that ^ code doesn't
        //run fast enough, we can just use this \/ code instead.
        /*
        a = Math.abs(a);
        b = Math.abs(b);
        c = Math.abs(c);
        d = Math.abs(d);

        if (d > a && d > b && d > c) {
            return d;
        }
        if (c > a && c > b && c > d) {
            return c;
        }
        if (b > a && b > c && b > d) {
            return b;
        }
        return a;
        */
    }

    public void resetEncoders() {
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
