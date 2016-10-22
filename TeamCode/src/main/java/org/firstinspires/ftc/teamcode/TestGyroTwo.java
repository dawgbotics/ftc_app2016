/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p/>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p/>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "gyrotest2", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice

public class TestGyroTwo extends OpMode {

    Wire gyro;
    int input = 0;
    byte[] high;
    int i = 0;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        gyro = new Wire(hardwareMap, "gyro", 2 * 0x6A);
        gyro.write(0x39, 0x00);
        gyro.write(0x23, 0x00);
        gyro.write(0x20, 0x6F);
    }

    @Override
    public void loop() {
        /*gyro.requestFrom(0x28 | (1 << 7), (byte) 6);
        //took out timeout
        int xlg = readVal(gyro, telemetry);
        int xhg = readVal(gyro, telemetry);
        int ylg = readVal(gyro, telemetry);
        int yhg = readVal(gyro, telemetry);
        int zlg = readVal(gyro, telemetry);
        int zhg = readVal(gyro, telemetry);

        int x = (int) (xhg << 8 | xlg);
        int y = (int) (yhg << 8 | ylg);
        int z = (int) (zhg << 8 | zlg);

        gyro.requestFrom(0b0101001, (byte) 1);
        int x2 = readVal(gyro, telemetry);
        gyro.endWrite();*/

        /*if(time.milliseconds() > 300) {
            time.reset();
            gyro.requestFrom(0b0101101, 1);
        }

        if (gyro.responseCount() > 0) {
            gyro.getResponse();
            if (gyro.isRead()) {
                int z2 = gyro.read();
                gyro.endWrite();
                i = i + 1;
                telemetry.addData("z2", z2);
                telemetry.addData("lol", i);
                telemetry.update();
            }
        }
        if (gyro.read() != 0) {
                if (input == 0) {
                    gyro.requestFrom(0xAD, 1);
                    gyro.requestFrom(0xAC, 1);
                }
                if(input==1){


                }


            }
        */
        if (gyro.responseCount() > 0) {
            gyro.getResponse();
            int regNumber = gyro.registerNumber();
            if (gyro.isRead()) {
                int regCount = gyro.available();
                switch (regNumber) {
                    case 0xA7:
                        if (regCount == 1) {
                            int status = gyro.read();
                            //telemetry.addData("status", status);
                            if (status != 0) {
                                gyro.requestFrom(0xAD, 2);             // Get colors
                            } else {
                                gyro.requestFrom(0xA7, 1);             // Keep polling
                            }
                        } else {
                            telemetry.addData("Error", regNumber + " length 1 != " + regCount);
                        }
                        break;
                    case 0xAD:// Keep polling
                        gyro.requestFrom(0xA7, 1);  // Check register count
                        int z = gyro.read();
                        i++;
                        telemetry.addLine();
                        telemetry.addData("z2", z);
                        telemetry.addData("lol", i);
                        break;
                    default:
                        telemetry.addData("Error", "Unexpected register " + regNumber);
                        break;
                }
            }
        }

    }

    @Override
    public void stop() {
        gyro.close();
    }

    @Override
    public void start() {
        gyro.requestFrom(0xA7, 1);             // Keep polling
    }

    public int readVal(Wire gyro, Telemetry telemetry) {
        while (gyro.responseCount() < 1) {
        }
        return gyro.read();

    }
}
