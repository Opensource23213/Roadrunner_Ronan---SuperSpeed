package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.sensors;
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

@TeleOp(name="TyneyCompetitionCode", group="ABC Opmode")
//@Disabled
public class TyneyCompetitionCode extends LinearOpMode {


    // Declare OpMode members.
     IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left = null;
    private Encoder vertwheel;
    private DcMotor front_right = null;
    private DcMotor rear_left = null;
    private DcMotor rear_right = null; //also has encoder for vertical deadwheel

    private DcMotor Arm1 = null;

    private DcMotor Arm2 = null;

    private Servo elbow1;
    private Servo mustaches;
    private Servo ilifty;
    private Servo grabby;
    private Servo wristy;
    private DigitalChannel limit;

    private CRServo spinny;
    private Servo flippy;
    private DcMotor ArmPos = null;

    private Servo lifty = null;
    private Servo shooty;
    private DistanceSensor sensorDistance;
    private DistanceSensor IntakeDis;
    private DistanceSensor sensorDistance2;
    private DistanceSensor flippydis;
    @Override
    public void runOpMode() {

        // Hardware Maps
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(IMU.class, "imu");

        // Motors
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        vertwheel = new Encoder(hardwareMap.get(DcMotorEx.class, "front_right"));
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");

        //Arm Servos
        elbow1 = hardwareMap.get(Servo.class, "elbow1"); //Elbow
        grabby = hardwareMap.get(Servo.class, "grabby"); //Grabber
        wristy = hardwareMap.get(Servo.class, "wristy"); //Wrist

        //Arm Encoder
        ArmPos = hardwareMap.get(DcMotor.class, "ArmPos");
        limit = hardwareMap.get(DigitalChannel.class, "limit");

        //Intake Servos
        spinny = hardwareMap.get(CRServo.class, "spinny"); //Vertical wheels
        flippy = hardwareMap.get(Servo.class, "flippy"); //Flipper
        ilifty = hardwareMap.get(Servo.class, "lifty2"); //Intake lifter
        mustaches = hardwareMap.get(Servo.class, "mustaches"); //Whiskers

        //Drone launcher servos
        shooty = hardwareMap.get(Servo.class, "shooty"); //Rubber band releaser
        lifty = hardwareMap.get(Servo.class, "lifty"); //Lifts the airplane launcher

        //Distance sensor stuff
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensorDistance2");
        flippydis = hardwareMap.get(DistanceSensor.class, "flippydis");
        IntakeDis = hardwareMap.get(DistanceSensor.class, "IntakeDis");
        Rev2mDistanceSensor IntakeDistime = (Rev2mDistanceSensor) IntakeDis;
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) sensorDistance2;
        Rev2mDistanceSensor sensorTimeOfFlight3 = (Rev2mDistanceSensor) flippydis;

        // servo position modifiers
        double mult = .22;
        double serAdjust = 3;

        //Arm_Encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));

        // Stuff like set direction
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        vertwheel.setDirection(Encoder.Direction.REVERSE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm1.setDirection(DcMotor.Direction.FORWARD);
        Arm2.setDirection(DcMotor.Direction.REVERSE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow1.setDirection(Servo.Direction.REVERSE);
        grabby.setDirection(Servo.Direction.REVERSE);
        lifty.setDirection(Servo.Direction.REVERSE);
        float wristOut = (float) ((float) .11 + serAdjust/355); // Used for A position
        float wristIn = (float) ((float) .6 + serAdjust/355); // Used for pick position
        wristy.setPosition(wristIn * mult); // Initiates the wrist
        double grabIn = .97; // Grabber closed
        double grabOut = .75; // One pixel grabbed
        double grabOut2 = .6; // Two pixels grabbed
        grabby.setPosition(grabIn); // initiates gripper
        float servoDegree = 1 / 355; // Multpilier for servos to convert their position to an angle
        spinny.setDirection(CRServo.Direction.FORWARD);  //intake
        double pow = 0;
        flippy.setPosition(.5); //Pixel flipper servo
        shooty.setPosition(.5); // initiates rubber band position
        double armInit = 2; // If the arm gets reset, it modifies this variable
        lifty.setPosition(0); // initiates drone launcher lift
        mustaches.setPosition(.28); // initiates whiskers
        ilifty.setPosition(.5); // initiates intake lifter
        double counter8 = 2; // Counter that helps hang efficiency


        // Define hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        

        telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
        // Wait for the game to start (driver presses PLAY)
        // Buttons for gamepad1 are capitalized while gamepad2 buttons aren't
        waitForStart();
        runtime.reset();



        // Random Bustle of Variables for buttons and other stuff
        double ticks = 22.76; // ticks per degree in encoder
        float deadcirc = (float) 4.329; // Dead wheel circumference in inches
        double ticksinch = .0029687175; // Dead Wheel Ticks per robot inch //12.5 @2.2, 15.5 @11.5
        double max; // Used later to make sure each motor's power is not greater than 1
        double power_level = 1; // Multiplier for chassis speed
        double Pi = 3.1415926 / 2; // Pi divided by 2
        int xPress1 = 1; // Tracks Gamepad 1's X button
        int xpress = 1; // Tracks Gamepad 2's X button
        int apress = 1; // Tracks Gamepad 2's A button
        int bPress = 1; // Tracks Gamepad 1's B button
        int bpress = 1; // Tracks Gamepad 2's B button
        int yPress = 1; // Tracks Gamepad 1's Y button
        int ypress = 1; // Tracks Gamepad 2's Y button
        double position = 0; // Tells the robot which arm position it should be at
        double serPosition = .91; // Gives the elbow position
        int counter = 0; // Counter used for B position movement
        int counter3 = 0; // Counter for the pick position
        double elbowTyney = 0; // The angle of the elbow when scoring
        double flippydoo = .5; // Setting the position of the flipper
        shooty.setPosition(.5);  // Sets the position for the plane launcher
        double armAngle = 0; // Initiates the variable for the Arm's degrees
        double shootcount = 0; // Counter for the drone launcher
        double pixelbut = 0; // Variable for the quickdrop button
        double hey = 1; // Variable for the intake opening process
        double xopen = 0; // Variable to track the x being open
        double ymove = 1; // Tracks if the robot is at y position
        double count = 1; // Helps with the second stage closing of the mustaches while intaking
        ElapsedTime timer = new ElapsedTime(SECONDS); // Used for time tracking
        double pypress = 1; // Helps with the backup after the pixel drop
        double sypress = 0; // Helps with the pixel drop
        double countback = 0; // Helps with the stages of movement in the pixel drop
        double topose = 2; // Helps with changing between moving to position and joystick
        double end = 1; // Helps with the hang button
        double rsb = 1; // Helps with the hang button
        double pixgrab = 1; // Used for the run code after button's released for X
        double pixgrab2 = 1; // Helps with the drop stages
        double typress = 1; // Helps with the quickdrop chassis movement
        double yes = 1; // Helps with strafing
        double once = 1; // Finds the angle when you initiate a strafe
        double roboyaw = 1; // Tracks the robots yaw
        double onegrab = 1; // For the intaking one pixel process
        double onegrab2 = 1; // Also for intaking one pixel process
        double counter9 = 0; // Used to offset the pick
        double once1 = 2; // Helps with moving pixels on board
        double robodis = 1;
        double startarmang = 1;
        double fieldoffset = 0;
        double offset = 0;
        double initiate = 1;
        double mechoffset = 0;
        double rude = 1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double dis = (sensorDistance.getDistance(DistanceUnit.INCH) + sensorDistance2.getDistance(DistanceUnit.INCH))/2;
            double flipdis = flippydis.getDistance(DistanceUnit.INCH);
            double wheeldis = vertwheel.getCurrentPosition() * ticksinch;
            boolean Intakepix = IntakeDis.getDistance(DistanceUnit.MM) < 67;
            // servo position in degrees
            double elbowDegree = elbow1.getPosition() * 355;
            double wristDegree = (wristy.getPosition() * 355 + serAdjust) * mult;

            //Variables Like Arm and Yaw
            // Setup a variable for each drive wheel to save power level for telemetry
            double frontleftPower;
            double frontrightPower;
            double rearleftPower;
            double rearrightPower;
            if(gamepad2.ps){
                if(gamepad2.left_trigger > .4){
                    topose = 4;
                }else {
                    topose = 3;
                }
            }

            armAngle = ArmPos.getCurrentPosition() / ticks - 22;



            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y / (gamepad1.left_trigger + 1);  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x / (gamepad1.left_trigger + 1);
            double yaw = gamepad1.right_stick_x / (gamepad1.left_trigger + 1);

            // joysticks on gamepad2 for arm
            double arm = gamepad2.left_stick_y/1.5; //power reduced to 25%
            double elbow = gamepad2.right_stick_y;

            // Eliminate stick drift
            if (gamepad2.left_stick_y < .05 && gamepad2.left_stick_y > -.05) {
                arm = 0;
            }

            // Retrieve Rotational Angles and Velocities


            // Check to see if heading reset is requested
            // Reset Field Centric

            if (gamepad1.ps) {
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
            if (gamepad1.left_trigger > .4 && gamepad1.a){
                flippydoo = 0;
                flippy.setPosition(0 );
            }
            else if(gamepad1.right_trigger > .4 && gamepad1.a){ // Shoots the drone
                if(shootcount < 2){
                    shootcount += 1;
                }else{
                    shooty.setPosition(.75);
                }
                lifty.setPosition(.2);
            }else if (gamepad1.a) { // Stops the intake
                ilifty.setPosition(.5);
                if(xpress < 2) {
                    flippydoo = .39;
                }
                spinny.setPower(0);
                yPress = 1;
                mustaches.setPosition(.332);

            }else{
                shooty.setPosition(.5);
                lifty.setPosition(0);
                shootcount = 0;
            }
            if (gamepad1.b){
                onegrab = 2;
            }
            if(!gamepad1.b && onegrab == 2){ // Code to intake one pixel at a time
                if (onegrab2 == 1){
                    ilifty.setPosition(.5);
                    flippydoo = .475;
                    mustaches.setPosition(.68);
                    spinny.setPower(1);
                    onegrab2 = 2;
                }else {
                    ilifty.setPosition(.5);
                    flippydoo = .475;
                    mustaches.setPosition(.28);
                    spinny.setPower(1);
                    onegrab2 = 1;
                }
                onegrab = 1;
            }
            if (gamepad1.x){
                pixgrab = 2;
            }
            if(!gamepad1.x && pixgrab == 2){ // Code for grabbing to pixels with mustaches and not intaking
                if (pixgrab2 == 1){
                    mustaches.setPosition(.68);
                    spinny.setPower(-1);
                    pixgrab2 = 2;
                }else {
                    mustaches.setPosition(.426666);
                    spinny.setPower(0);
                    pixgrab2 = 1;
                }
                pixgrab = 1;
            }
            //Set Drive Mode
           /* if (gamepad1.x && xPress1 == 1) {  // Field Centric
                Drive_Mode = 1;
                xPress1 = 2;
            } /*else if (gamepad1.x && xPress1 == 1) {  // POV
                Drive_Mode = 0;
                xPress1 = 1;
            }*/


            //deactivate and activate Intake

            if (gamepad1.right_trigger > .4 && gamepad1.y) { // Reverses the intake
                flippydoo = .44;
                spinny.setPower(-1);
            } else if (gamepad1.y){ // Code for normal intaking of pixels
                spinny.setPower(1);
                flippydoo = .475;
                if (hey == 1){
                    if(yPress == 1){
                        ilifty.setPosition(.3);
                        mustaches.setPosition(.68);
                        yPress = 2;
                        hey = 2;
                        timer.reset();
                        rude = 2;
                    }else{
                        mustaches.setPosition(.42);
                        count = 2;
                        yPress = 1;
                        hey = 2;
                    }
                }
            }
            if (rude == 2 && Intakepix && timer.time() > .5){
                mustaches.setPosition(.28);
                ilifty.setPosition(.5);
                yPress = 1;
                rude = 1;
            }
            if (count >= 2 && count <= 25){
                count += 1;
            }else if(count >= 26){ // Code for the other stage of the mustaches closing and intake coming down
                mustaches.setPosition(.28);
                ilifty.setPosition(.5);
                count = 1;
            }
            if (!gamepad1.y){
                hey = 1;
            }
            flippy.setPosition(flippydoo); //Moves flipper to position


            if(gamepad2.left_bumper){
                grabby.setPosition(grabOut2);
            }
            // Send calculated power to wheels
            if (gamepad2.x) {
                if(gamepad2.right_trigger > .4){
                    typress = 2;
                }
                else if (gamepad2.left_trigger > .4) {  //initiates the pick
                    xpress = 2;
                    grabby.setPosition(grabIn);
                    wristy.setPosition((float) ((float) .67 * mult));
                    flippydoo = .23;
                    flippy.setPosition(.23);

                } else {  //Gripper open and close
                    pypress = 2;
                }
            }
            if (!gamepad2.x && pypress == 2){ // Code to drop pixels
                if (sypress == 0){
                    sypress = 1;
                    grabby.setPosition(grabOut);
                    countback = 0;
                    pypress = 3;
                }
                else if (sypress == 1){
                    sypress = 2;
                    grabby.setPosition(grabIn);
                    countback = 0;
                    pypress = 3;
                }
            }
            if(!gamepad2.x && typress == 2){ // Starts code for quick drop
                grabby.setPosition(grabOut);
                countback = 0;
                typress = 4;
            }
            // D pad used to move the robot
            if(xPress1 == 1 && bPress == 1){
                yes = 2;
                if (pypress == 3){ // Backwards movement for after regular drop
                    while (countback < 8){
                        if (ypress == 3) {
                            front_left.setPower(.6);
                            front_right.setPower(.6);
                            rear_left.setPower(.6);
                            rear_right.setPower(.6);
                        }else{
                            front_left.setPower(-.6);
                            front_right.setPower(-.6);
                            rear_left.setPower(-.6);
                            rear_right.setPower(-.6);
                        }
                        sleep(100);
                        countback = 8;

                    }
                    if (countback > 7) { // sets movement back to the stick
                        axial = -gamepad1.left_stick_y / (gamepad1.left_trigger + 1);  // Note: pushing stick forward gives negative value
                        lateral = gamepad1.left_stick_x / (gamepad1.left_trigger + 1);
                        yaw = gamepad1.right_stick_x / (gamepad1.left_trigger + 1);
                        yes = 1;
                        pypress = 1;
                    }
                }else if (typress == 4){ // Chassis movement for quickdrop process
                    while (countback < 5){ // Moves back
                        if (ypress == 3) {
                            front_left.setPower(.5);
                            front_right.setPower(.5);
                            rear_left.setPower(.5);
                            rear_right.setPower(.5);
                        }else{
                            front_left.setPower(-.5);
                            front_right.setPower(-.5);
                            rear_left.setPower(-.5);
                            rear_right.setPower(-.5);
                        }
                        sleep(180);
                        countback = 6;

                    }
                    while (countback < 8){ // Moves forward and drops pixel
                        if (ypress == 3) {
                            front_left.setPower(-.5);
                            front_right.setPower(-.5);
                            rear_left.setPower(-.5);
                            rear_right.setPower(-.5);
                        }else{
                            front_left.setPower(.5);
                            front_right.setPower(.5);
                            rear_left.setPower(.5);
                            rear_right.setPower(.5);
                        }
                        sleep(200);
                        countback = 8;

                    }
                    if (countback > 7) { // Moves back
                        grabby.setPosition(grabIn);
                        if (ypress == 3) {
                            front_left.setPower(.5);
                            front_right.setPower(.5);
                            rear_left.setPower(.5);
                            rear_right.setPower(.5);
                        } else {
                            front_left.setPower(-.5);
                            front_right.setPower(-.5);
                            rear_left.setPower(-.5);
                            rear_right.setPower(-.5);
                        }
                        sleep(200);
                        countback = 9;
                    }
                    if (countback > 8){ // Set control back to the sticks
                        axial = -gamepad1.left_stick_y / (gamepad1.left_trigger + 1);  // Note: pushing stick forward gives negative value
                        lateral = gamepad1.left_stick_x / (gamepad1.left_trigger + 1);
                        yaw = gamepad1.right_stick_x / (gamepad1.left_trigger + 1);
                        yes = 1;
                        typress = 1;
                    }
                }
                else if(gamepad1.dpad_left){ // strafe left
                    if (ymove == 1) {
                        if (gamepad1.right_trigger > .4) {
                            axial = 0;
                            lateral = -.8;
                            yaw = 0;
                        } else {
                            axial = 0;
                            lateral = -.5;
                            yaw = 0;
                        }
                    }else{
                        if (gamepad1.right_trigger > .4) {
                            axial = 0;
                            lateral = .8;
                            yaw = 0;
                        }else {
                            axial = 0;
                            lateral = .5;
                            yaw = 0;
                        }
                    }
                }else if(gamepad1.dpad_right){ // strafe right
                    if(ymove == 1) {
                        if (gamepad1.right_trigger > .4) {
                            axial = 0;
                            lateral = .8;
                            yaw = 0;
                        } else {
                            axial = 0;
                            lateral = .5;
                            yaw = 0;
                        }
                    }else {
                        if (gamepad1.right_trigger > .4) {
                            axial = 0;
                            lateral = -.8;
                            yaw = 0;
                        } else {
                            axial = 0;
                            lateral = -.5;
                            yaw = 0;

                        }
                    }
                }else if(gamepad1.dpad_up){ // strafe forward
                    if(ymove == 1) {
                        if (gamepad1.right_trigger > .4) {
                            axial = .8;
                            lateral = 0;
                            yaw = 0;
                        } else {
                            axial = .5;
                            lateral = 0;
                            yaw = 0;
                        }
                    }else {
                        if (gamepad1.right_trigger > .4) {
                            axial = -.8;
                            lateral = 0;
                            yaw = 0;
                        } else {
                            axial = -.5;
                            lateral = 0;
                            yaw = 0;
                        }
                    }
                }else if(gamepad1.dpad_down) { // Strafe backward
                    if(ymove != 1) {
                        if (gamepad1.right_trigger > .4) {
                            axial = .8;
                            lateral = 0;
                            yaw = 0;
                        } else {
                            axial = .5;
                            lateral = 0;
                            yaw = 0;
                        }
                    }else {
                        if (gamepad1.right_trigger > .4) {
                            axial = -.8;
                            lateral = 0;
                            yaw = 0;
                        } else {
                            axial = -.5;
                            lateral = 0;
                            yaw = 0;
                        }
                    }
                }else{ // set control to the sticks
                    axial = -gamepad1.left_stick_y / (gamepad1.left_trigger + 1);  // Note: pushing stick forward gives negative value
                    lateral = gamepad1.left_stick_x / (gamepad1.left_trigger + 1);
                    yaw = gamepad1.right_stick_x / (gamepad1.left_trigger + 1);
                    yes = 1;
                }
            }
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            if (initiate == 1){
                offset = orientation.getYaw(AngleUnit.DEGREES);
                initiate = 2;
            }
            if ((Math.abs(orientation.getYaw(AngleUnit.DEGREES)) < 20 || Math.abs(orientation.getYaw(AngleUnit.DEGREES)) > 340) && Math.abs(offset) < 320 && Math.abs(offset) > 40){
                fieldoffset = offset;
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetDeviceConfigurationForOpMode();
                imu.resetYaw();
            }else{
                offset = orientation.getYaw(AngleUnit.DEGREES) + fieldoffset;
                if (offset < -360){
                    offset += 360;
                }else if (offset > 360){
                    offset -= 360;
                }
            }
// Field Centric Calculations
            double leftFrontPower = (axial + lateral + yaw) * power_level;
            double rightFrontPower = (axial - lateral - yaw) * power_level;
            double leftBackPower = (axial - lateral + yaw) * power_level;
            double rightBackPower = (axial + lateral - yaw) * power_level;

            if (yes == 2){ // while a power modifying button is being used
                if (once == 1) { // finds the starting angle
                    roboyaw = orientation.getYaw(AngleUnit.DEGREES);
                    once = 2;
                }
                double angdif = orientation.getYaw(AngleUnit.DEGREES) - roboyaw;
                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                // Modifies power based on the angle difference
                leftFrontPower = (axial + lateral + (yaw + angdif * .02)) * power_level;
                rightFrontPower = (axial - lateral - (yaw + angdif * .02)) * power_level;
                leftBackPower = (axial - lateral + (yaw + angdif * .02)) * power_level;
                rightBackPower = (axial + lateral - (yaw + angdif * .02)) * power_level;

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
            }else{ // If the sticks are being used
                once = 1;
                double yaw_rad = orientation.getYaw(AngleUnit.RADIANS) + Pi;
                if (fieldoffset != 0 && (orientation.getYaw(AngleUnit.DEGREES) + fieldoffset > 360 || orientation.getYaw(AngleUnit.DEGREES) + fieldoffset < -360)){
                    mechoffset = orientation.getYaw(AngleUnit.DEGREES) + fieldoffset;
                    if (mechoffset < 0){
                        mechoffset += 360;
                    }else{
                        mechoffset -= 360;
                    }
                    yaw_rad = Math.toRadians(Math.abs(mechoffset)) + Pi;
                }else if(fieldoffset != 0){
                    mechoffset = orientation.getYaw(AngleUnit.DEGREES) + fieldoffset;
                    yaw_rad = Math.toRadians(mechoffset) + Pi;
                }
                double temp = axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
                lateral = -axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
                //double temp = axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
                //lateral = -axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
                axial = temp;

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                leftFrontPower = (axial + lateral + yaw) * power_level;
                rightFrontPower = (axial - lateral - yaw) * power_level;
                leftBackPower = (axial - lateral + yaw) * power_level;
                rightBackPower = (axial + lateral - yaw) * power_level;

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
            }
            //Arm code Shoulder

            front_left.setPower(leftFrontPower);
            front_right.setPower(rightFrontPower);
            rear_left.setPower(leftBackPower);
            rear_right.setPower(rightBackPower);

            if (gamepad2.a) { //Low score position

                if(pixelbut == 0){
                    topose = 2;
                    xopen = 0;
                    ymove = 1;
                    ypress = 1;
                    apress = 2;
                    position = 3;
                }
            }
            if(gamepad2.right_stick_button){
                rsb = 2;
            }
            if(!gamepad2.right_stick_button && rsb == 2){ // Helps hang efficiency
                if(end == 1){
                    end = 2;
                    position = 3;
                    topose = 2;
                    rsb = 1;
                }else {
                    end = 1;
                    topose = 1;
                    rsb = 1;
                }
            }
            if (gamepad2.b) { // Pounce position
                topose = 2;
                ypress = 1;
                ymove = 1;
                xopen = 0;
                serPosition = .91;
                wristy.setPosition(.67 * mult);
                grabby.setPosition(grabIn);
                position = -10;
                counter8 = 0;
                bpress = 2;

            }
            if (gamepad2.y) {// Play position 2
                if (xpress == 1){
                    topose = 2;
                    position = 125;
                    ymove = 2;
                    ypress = 2;
                }
            }

            // pick pixels
            else if (xpress == 2) { // raises flipper
                flippy.setPosition(.18);
                flippydoo = .18;
                timer.reset();
                xpress = 3;
                counter3 = 0;
            } else if (xpress == 3 && timer.time() > .25) { // Sticks gripper through the pixels
                serPosition = 1;
                xpress = 4;
                counter3 = 0;
            } else if (xpress == 4 && timer.time() > .55) { // Opens gripper
                sypress = 0;
                grabby.setPosition(grabOut2);
                xpress = 5;
                counter3 = 0;
            } else if (xpress == 5 && timer.time() > .75) { // Puts flipper back down
                flippydoo = .5;
                flippy.setPosition(flippydoo);
                xpress = 6;
                counter3 = 0;
            } else if (xpress == 6 && timer.time()>.85) { // brings elbow back and sets wrist position
                serPosition = .91;
                xpress = 1;
                counter3 = 0;
                wristy.setPosition((float) ((float) .67 * mult));
            }

            if (gamepad2.right_bumper) { // Drop both pixels
                grabby.setPosition(grabIn);
            }else{ // kinematics for a
                if (armAngle > -10 && ypress != 2 && xpress < 2 && serPosition == 0 && ypress != 3) { //
                    wristy.setPosition(((68 + serAdjust + -5 * 1.4 - armAngle * 1.4) / 355) * mult);
                    arm *= .7;
                }
            }

            // kinematics for y
            if (ypress == 3 && armAngle >= -10 && xpress < 2) {
                wristy.setPosition(((172 + serAdjust + 99 * 1.2 - armAngle * 1.2) / 355) * mult);  //
                arm /= 3;
            }
            if (ypress != 3 && xpress >= 2) { // where the wrist stays if it isn't at "a" or "y"
                wristy.setPosition((float) ((float) .67 * mult));
            }
            if (end == 2){ // Speeds up arm for hang
                arm *= 1.5;
            }
            // b movement
            if (bpress == 2 && counter < 7) {
                counter += 1;
            } else if (apress == 3 && counter < 7) {
                counter += 1;
            } else {
                counter = 0;
                bpress = 1;
                if (apress == 3) {
                    topose = 2;
                    apress = 1;
                }
                if (topose == 1) {
                    if ((gamepad2.left_stick_y > .2 || gamepad2.left_stick_y < -.2) && ymove == 1) { // Normal arm movement
                        Arm1.setPower(arm);
                        Arm2.setPower(arm);
                        position = armAngle;
                    }else if((gamepad2.left_stick_y > .2 || gamepad2.left_stick_y < -.2) && ymove != 1){ // Reversed arm movement for at Y
                        Arm1.setPower(-arm);
                        Arm2.setPower(-arm);
                        position = armAngle;
                    }
                    else{
                        Arm1.setPower(0);
                        Arm2.setPower(0);

                    }
                } else if(topose == 2){ // Arm move to position code
                    if (position < armAngle + 1 && position > armAngle - 1) {// Stop arm movement within a 3 degree range
                        Arm1.setPower(0);
                        Arm2.setPower(0);
                        if (apress == 2) {
                            serPosition = 0;
                            wristy.setPosition(wristOut * mult);
                            apress = 3;
                        }
                        if(end == 1 || end == 2 && gamepad2.left_stick_y > .2 || end == 2 && gamepad2.left_stick_y < -.2) {
                            if (counter8 < 2) {
                                counter8 += 1;
                            } else {
                                if (end == 2){
                                    topose = 2;
                                }
                                else {
                                    topose = 1;
                                }
                            }
                        }


                    } else if (position > armAngle + 11 || position < armAngle - 11) {//  Far and fast arm move into position within an infinite range
                        if (position < armAngle) {
                            Arm1.setPower(1);
                            Arm2.setPower(1);
                        }
                        if (position > armAngle) {
                            Arm1.setPower(-1);
                            Arm2.setPower(-1);
                        }
                        if (ypress == 2) {
                            grabby.setPosition(grabOut2);
                            serPosition = elbowTyney;
                            ypress = 3;
                        }
                    } else { //Close and slow arm move into position if arm is in a 16 degree range
                        if (position < armAngle) {
                            Arm1.setPower(.2);
                            Arm2.setPower(.2);
                        }
                        if (armAngle < 10){
                            if (position > armAngle) {
                                Arm1.setPower(-.1);
                                Arm2.setPower(-.1);
                            }
                        }
                        if (position > armAngle) {
                            Arm1.setPower(-.2);
                            Arm2.setPower(-.2);
                        }

                    }
                }else if(topose == 3){
                    Arm1.setPower(.1);
                    Arm2.setPower(.1);
                    if(!limit.getState()){
                        ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        position = -10;
                        topose = 2;
                    }

                }else{
                    Arm1.setPower(.1);
                    Arm2.setPower(.1);
                    if(!limit.getState()){
                        ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Arm1.setPower(0);
                        Arm2.setPower(0);
                    }

                }
            }
            // Arm code Elbow

            elbow1.setPosition(serPosition);


            // Telemetry
            telemetry.addData("Status", fieldoffset);
            telemetry.addLine("");
            telemetry.addLine("Arm Values:");
            telemetry.addLine("");
            telemetry.addData("Shoulder Arm", "Angle: " + armAngle);
            telemetry.addData("Shoulder Arm", "Angle: " + IntakeDis.getDistance(DistanceUnit.MM));
            telemetry.addData("Elbow Joint", "Value: " + elbow);
            telemetry.addData("Wrist", (wristy.getPosition() * 355 + serAdjust)* mult);
            telemetry.addLine("");
            telemetry.addLine("Drive Train Values:");
            telemetry.addLine("");
            telemetry.addData("Forward Distance: ", dis);
            telemetry.addData("Front left/Right: ", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right: ", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Encoder: ", wheeldis);

            telemetry.addData("Grabby Position: ", mustaches.getPosition());
            telemetry.addData("Grabby Position: ", limit.getState());
            telemetry.addLine("");
            telemetry.addLine("Angles:");
            telemetry.addLine("");
            telemetry.addData("Yaw (Z): ", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X): ", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y):  ", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("flippydis: ", flipdis);
            telemetry.update();

        }
    }
}



