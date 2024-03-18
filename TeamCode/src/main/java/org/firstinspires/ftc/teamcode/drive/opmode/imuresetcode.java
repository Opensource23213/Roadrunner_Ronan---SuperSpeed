package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Encoder;

/*
  This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
  the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
  of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
  class is instantiated on the Robot Controller and executed.

  This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
  It includes all the skeletal structure that all linear OpModes contain.

  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
  Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="imuresetcode", group="ABC Opmode")
@Disabled
public class imuresetcode extends LinearOpMode {


    // Declare OpMode members.
     IMU imu;
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor rear_left = null;
    private DcMotor rear_right = null;
    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu"); //TODO 2 Enter IMU name
        front_left = hardwareMap.get(DcMotor.class, "front_left"); //TODO 2 Enter motor names
        front_right = hardwareMap.get(DcMotor.class, "front_right"); //TODO 2 Enter motor names
        rear_left = hardwareMap.get(DcMotor.class, "rear_left"); //TODO 2 Enter motor names
        rear_right = hardwareMap.get(DcMotor.class, "rear_right"); //TODO 2 Enter motor names
        front_left.setDirection(DcMotor.Direction.REVERSE); //TODO 2 Customize Motor Direction
        front_right.setDirection(DcMotor.Direction.FORWARD);//TODO 2 Customize Motor Direction
        rear_left.setDirection(DcMotor.Direction.REVERSE);//TODO 2 Customize Motor Direction
        rear_right.setDirection(DcMotor.Direction.FORWARD);//TODO 2 Customize Motor Direction
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Define hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN; //TODO Change based on your control hub orientation
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        waitForStart();



        // Random Bustle of Variables for buttons and other stuff
        double power_level = 1; //TODO 3 Change based on your robot's speed multiplier
        double Pi = 3.1415926 / 2;
        double fieldoffset = 0;
        double offset = 0;
        double initiate = 1;
        double mechoffset = 0;
        double max;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y / (gamepad1.left_trigger + 1);  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x / (gamepad1.left_trigger + 1);
            double yaw = gamepad1.right_stick_x / (gamepad1.left_trigger + 1);

            if (gamepad1.ps) { //TODO Customize what controller button you want for this
                telemetry.addData("Yaw", "Resetting\n");
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetDeviceConfigurationForOpMode();
                imu.resetYaw();
                fieldoffset = 0;
                mechoffset = 0;
                initiate = 1;
            } else {
                telemetry.addData("Yaw", "Press Middle Button (Logic) on Gamepad to reset\n");
            }
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles(); // Gets the current angles of the roobt
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            if (initiate == 1) {
                offset = orientation.getYaw(AngleUnit.DEGREES); // Initiates the offset to the current angle
                initiate = 2;
            }
            //Checks if your robot angle is close to zero while your previouse angle isn't
            if ((Math.abs(orientation.getYaw(AngleUnit.DEGREES)) < 20 || Math.abs(orientation.getYaw(AngleUnit.DEGREES)) > 340) && Math.abs(offset - fieldoffset) < 320 && Math.abs(offset - fieldoffset ) > 40) {
                fieldoffset = offset;
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
                orientation = imu.getRobotYawPitchRollAngles();
                offset = orientation.getYaw(AngleUnit.DEGREES) + fieldoffset;
            } else {
                offset = orientation.getYaw(AngleUnit.DEGREES) + fieldoffset;
                if (offset < -360) { // If the offset is over 360 it makes it to its smaller corresponding value
                    offset += 360;
                } else if (offset > 360) {
                    offset -= 360;
                }
            }
            // Field Centric Calculations
            double yaw_rad = orientation.getYaw(AngleUnit.RADIANS) + Pi;
            //Checks if it has been offset and if adding that offset will make it greater than 360
            if (fieldoffset != 0 && (orientation.getYaw(AngleUnit.DEGREES) + fieldoffset > 360 || orientation.getYaw(AngleUnit.DEGREES) + fieldoffset < -360)) {
                mechoffset = orientation.getYaw(AngleUnit.DEGREES) + fieldoffset;
                if (mechoffset < 0) {
                    mechoffset += 360;
                } else {
                    mechoffset -= 360;
                }
                yaw_rad = Math.toRadians(Math.abs(mechoffset)) + Pi; // Adds the offset to the field centric
            } else if (fieldoffset != 0) {//If not greater than 360 it just adds the offset straight up
                mechoffset = orientation.getYaw(AngleUnit.DEGREES) + fieldoffset;
                yaw_rad = Math.toRadians(mechoffset) + Pi;
            }
            double temp = axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
            lateral = -axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
            axial = temp;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = (axial + lateral + yaw) * power_level;
            double rightFrontPower = (axial - lateral - yaw) * power_level;
            double leftBackPower = (axial - lateral + yaw) * power_level;
            double rightBackPower = (axial + lateral - yaw) * power_level;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
            max = Math.max(max, abs(leftBackPower));
            max = Math.max(max, abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            //Set the motor's speed
            front_left.setPower(leftFrontPower);
            front_right.setPower(rightFrontPower);
            rear_left.setPower(leftBackPower);
            rear_right.setPower(rightBackPower);

            // Telemetry
            telemetry.addLine("Drive Train Values:");
            telemetry.addLine("");
            telemetry.addData("Offset: ", fieldoffset);
            telemetry.addData("Front left/Right: ", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right: ", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Yaw (Z): ", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

    }
}





