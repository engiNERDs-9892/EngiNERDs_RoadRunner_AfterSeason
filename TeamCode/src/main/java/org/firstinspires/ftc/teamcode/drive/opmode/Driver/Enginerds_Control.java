package org.firstinspires.ftc.teamcode.drive.opmode.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="Enginerds_Control", group="Linear Opmode")
//@Disabled

public class Enginerds_Control extends LinearOpMode {

    // The private DcMotor _____; are used to identify a motor that can be used throughout the code

    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private DcMotor motorRiseyRise = null;
    private DcMotor motorSlideySlide = null;


    // How we call the Top Sensor (The one we don't use)
    TouchSensor touchT;

    // How we call the Bottom Sensor (The one we use)
    TouchSensor touchB;


    // The Servo ____; are used to identify a servo that can be used throughout the code.
    Servo servoFAL;
    Servo servoFAR;

    @Override
    public void runOpMode() {

        // HardwareMap Section (Used to talk to the driver hub for the configuration)

        // Motors

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorRiseyRise = hardwareMap.dcMotor.get("motorRiseyRise");
        motorSlideySlide = hardwareMap.dcMotor.get("motorSlideySlide");

        // Servos
        servoFAL = hardwareMap.servo.get("servoFAL");
        servoFAR = hardwareMap.servo.get("servoFAR");


        // Touch Sensor
        touchT = hardwareMap.touchSensor.get("touchT");
        touchB = hardwareMap.touchSensor.get("touchB");

        // Setting the motor Power for Driver Control
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorRiseyRise.setPower(0);
        motorSlideySlide.setPower(0);


        // Setting the motor Direction, so the motors rotate correctly (Default Direction = Forward)
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /////////////////////////////////
            // Variables for Driver Inputs //
            /////////////////////////////////

            // Variable used for Regular speed (To find the direction that the stick needs to be in)
            double max;

            // Variable used for Fast speed (To find the direction that the stick needs to be in)
            double fmax;

            // Variable used for Slow speed (To find the direction that the stick needs to be in)
            double smax;

            double RaiseorLower = gamepad2.left_stick_y;

            // The code below talks about the Y-axis (Up and Down / Forward and Backwards)

            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value

            // The code below talks about the X-axis (Left and Right)

            double lateral = -gamepad1.left_stick_x; // The bottom two are inverted because motor direction is changed

            // The code below talks about Z-Axis (Spinning around)

            double yaw = -gamepad1.right_stick_x;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////
            // Math for the Left Joystick //
            ////////////////////////////////


            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Regular speed
            double leftFrontPower = .6 * (axial + lateral + yaw);
            double rightFrontPower = .6 * (axial - lateral - yaw);
            double leftBackPower = .6 * (axial - lateral + yaw);
            double rightBackPower = .6 * (axial + lateral - yaw);

            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Fast speed
            double FleftFrontPower = (axial + lateral + yaw);
            double FrightFrontPower = (axial - lateral - yaw);
            double FleftBackPower = (axial - lateral + yaw);
            double FrightBackPower = (axial + lateral - yaw);

            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Slow speed
            double SleftFrontPower = .25 * (axial + lateral + yaw);
            double SrightFrontPower = .25 * (axial - lateral - yaw);
            double SleftBackPower = .25 * (axial - lateral + yaw);
            double SrightBackPower = .25 * (axial + lateral - yaw);


            ////////////////////////////////////////////////////////////////////////////////////////////
            // use LEFT joystick to go Forward/Backwards & left/Right, and RIGHT joystick to Rotate.///
            //////////////////////////////////////////////////////////////////////////////////////////


            // This calculates the direction & power for Regular Speed
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            // This calculates the direction & power for Fast Speed
            fmax = Math.max(Math.abs(FleftFrontPower), Math.abs(FrightFrontPower));
            fmax = Math.max(fmax, Math.abs(FleftBackPower));
            fmax = Math.max(fmax, Math.abs(FrightBackPower));

            // This calculates the direction & power for Fast Speed
            smax = Math.max(Math.abs(SleftFrontPower), Math.abs(SrightFrontPower));
            smax = Math.max(smax, Math.abs(SleftBackPower));
            smax = Math.max(smax, Math.abs(SrightBackPower));


            // sets the wheels to do whatever the calculation above tells it to do Fast Speed
            if (fmax > 1.0) {

                FleftFrontPower = fmax;
                FrightFrontPower = fmax;
                FleftBackPower = fmax;
                FrightBackPower = fmax;
            }

            // sets the wheels to do whatever the calculation above tells it to do for Slow Speed
            if (smax > 1.0) {
                SleftFrontPower = smax;
                SrightFrontPower = smax;
                SleftBackPower = smax;
                SrightBackPower = smax;
            }

            // sets the wheels to do whatever the calculation above tells it to do for Regular Speed
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            // Setting the power for Slow Speed
            if (gamepad1.left_trigger != 0) {
                motorFL.setPower(SleftFrontPower);
                motorBL.setPower(SleftBackPower);
                motorBR.setPower(SrightBackPower);
                motorFR.setPower(SrightFrontPower);

            }

            // Setting the power for Fast Speed
            else if (gamepad1.right_trigger != 0) {

                motorFL.setPower(FleftFrontPower);
                motorBL.setPower(FleftBackPower);
                motorBR.setPower(FrightBackPower);
                motorFR.setPower(FrightFrontPower);
            }

            // Setting the power for Regular Speed
            else {
                motorFL.setPower(leftFrontPower);
                motorBL.setPower(leftBackPower);
                motorFR.setPower(rightFrontPower);
                motorBR.setPower(rightBackPower);
            }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////



            //////////////////////////////////////////////////////////////
            /////      Servo Code for arms left and right        /////////
            //////////////////////////////////////////////////////////////

            // Close Claws

            if (gamepad2.a) {
                servoFAL.setPosition(0);
                servoFAR.setDirection(Servo.Direction.REVERSE);
                servoFAR.setPosition(0);
            }

            // Open Claws

            if (gamepad2.y) {
                servoFAL.setPosition(.15);
                servoFAR.setDirection(Servo.Direction.REVERSE);
                servoFAR.setPosition(.15);
            }


            //////////////////////////////////////////////////////////////////////
            /////   Motors for the Linear Slides (How to go up and Down) /////////
            //////////////////////////////////////////////////////////////////////

            // If you raised the Linear Slides

            if (RaiseorLower < -0.05){
                motorRiseyRise.setPower(RaiseorLower);
                motorSlideySlide.setPower(RaiseorLower);
            }


            // if you are trying to lower the linear Slide and the sensor is not triggered
            // Then lower the x-rail!

            if (RaiseorLower > 0.05 && !touchB.isPressed() || touchT.isPressed()){
                motorRiseyRise.setPower(RaiseorLower);
                motorSlideySlide.setPower(RaiseorLower);


                // If you are trying to lower the Linear Slide and it touches the sensor
                // the power will = 0

            if (RaiseorLower > 0.05 && touchB.isPressed() || touchT.isPressed()){
                motorRiseyRise.setPower(0);
                motorSlideySlide.setPower(0);
            }
        }

            // If you are not pushing on the joystick the power = 0
            // This is mainly to prevent stick drift
            if((RaiseorLower >= -0.05) && (RaiseorLower <= 0.05)){
                motorRiseyRise.setPower(0);
                motorSlideySlide.setPower(0);
            }
        }

    }
}

