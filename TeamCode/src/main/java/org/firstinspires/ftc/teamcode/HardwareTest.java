package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="hardware test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class HardwareTest extends LinearOpMode {

    //this program tests each system, with the motors and gyroscope going autonomously and
    //telling the operator via telemetry if they work. The rest goes until the operator
    //hits a button

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    Drive drive;
    DcMotor motorGun1;

    DcMotor motorLeftBack;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorRightFront;

    DcMotor motorIntake;

    DcMotor motorArm;
    DcMotor motorArm2;

    @Override
    public void runOpMode() {

        //creates a gyro object
        Gyro gyro = new Gyro(hardwareMap, "gyro");

        //creates all motors and servos
        motorGun1 = hardwareMap.dcMotor.get("gun 1");
        motorGun1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGun1.setDirection(DcMotorSimple.Direction.FORWARD);

        motorIntake = hardwareMap.dcMotor.get("intake");
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorArm = hardwareMap.dcMotor.get("catapult");
        motorArm2 = hardwareMap.dcMotor.get("catapult 2");
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm2.setDirection(DcMotorSimple.Direction.REVERSE);


        motorLeftBack = hardwareMap.dcMotor.get("back left");
        motorLeftFront = hardwareMap.dcMotor.get("front left");
        motorRightBack = hardwareMap.dcMotor.get("back right");
        motorRightFront = hardwareMap.dcMotor.get("front right");

        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //runs each motor and tells you if the encoder works
        motorLeftBack.setPower(0.618);
        sleep(500);
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0.618);
        sleep(500);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0.618);
        sleep(500);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0.618);
        sleep(500);
        motorRightFront.setPower(0);

        if (Math.abs(motorLeftBack.getCurrentPosition()) - 30 > 0) {
            telemetry.addData("leftBack: ", "good");
        } else {
            telemetry.addData("leftBack: ", "bad");
        }

        if (Math.abs(motorRightBack.getCurrentPosition()) - 30 > 0) {
            telemetry.addData("rightBack: ", "good");
        } else {
            telemetry.addData("rightBack: ", "bad");
        }

        if (Math.abs(motorLeftFront.getCurrentPosition()) - 30 > 0) {
            telemetry.addData("LeftFront: ", "good");
        } else {
            telemetry.addData("LeftFront: ", "bad");
        }

        if (Math.abs(motorRightFront.getCurrentPosition()) - 30 > 0) {
            telemetry.addData("RightFront: ", "good");
        } else {
            telemetry.addData("RightFront: ", "bad");
        }

        sleep(3000);

        waitForStart();

        //start gun test
        motorGun1.setPower(.5);

        while (!gamepad1.a) { //wait until the a button is pressed, then start the intake test
        }
        //end gun test
        motorGun1.setPower(0);
        sleep(1000);

        //start intake test
        motorIntake.setPower(.75);

        while (!gamepad1.a) { //wait until the a button is pressed, then start the catapult test
        }
        //end intake test
        motorIntake.setPower(0);
        sleep(1000);

        //start catapult test
        motorArm.setPower(.75);
        motorArm2.setPower(.75);

        while (!gamepad1.a) { //wait until the a button is pressed, then start the gyro test
        }
        //end catapult test
        motorArm.setPower(0);
        motorArm2.setPower(0);
        sleep(1000);

        //start gyro test
        gyro.readZ();
        int oldZ = gyro.getAngleZ();

        motorLeftBack.setPower(.75);
        motorLeftFront.setPower(.75);
        motorRightBack.setPower(.75);
        motorRightFront.setPower(.75);

        sleep(1000);

        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);

        gyro.readZ();
        int newZ = gyro.getAngleZ();

        int diff = Math.abs(newZ - oldZ);
        boolean gyroWorking = diff > 4;
        if (gyroWorking) {
            telemetry.addData("Gyro working, rotated: ", diff);
        }
        else {
            telemetry.addData("Gyro not working", "");
        }
    }
}
