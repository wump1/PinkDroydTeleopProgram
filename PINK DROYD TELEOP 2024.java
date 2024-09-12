package org.firstinspires.ftc.teamcode;


//imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop10393F", group="Linear OpMode")
public class Teleop10393F extends LinearOpMode {
    private static final double DELTA_TIME = 0.005; // Average time between main loop runs
    private static final double MIN_INPUT_THRESHOLD = 0.01; // Minimum proportional input before activating


    // Drive values
    private static final double STRAFE_YAW_BLEND = 0.1;


    // Arm values
    private static final double ARM_MAX = 850;
    private static final double ARM_MIN_TIME = 2; // Shortest amount of time to complete a min-to-max-swing
    private static final double ARM_PROPORTIONAL_PID = 20;


    // Bucket values
    private static final double BUCKET_FLOOR_POS = 0.3;
    private static final double BUCKET_ARM_M_1 = 0.001057437408;
    private static final double BUCKET_ARM_M_2 = -0.0008282613227;
    private static final double BUCKET_ARM_B_2 = 1.286516869;


    // Intake values
    private static final double INTAKE_POWER = 1;


    // Stud values
    private static final double STUD_MIN = 0;
    private static final double STUD_MAX = 0.35;
    
    // Hang values
    private static final double HANG_SPEED = 4000;


    @Override
    public void runOpMode() {
        // Initialize drive motors
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "leftF");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "rightF");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "leftB");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "rightB");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        // Initialize arm
        DcMotorEx arm = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPositionPIDFCoefficients(ARM_PROPORTIONAL_PID);
        double armTarget = 0;


        // Initialize bucket (3d Printed bucket)
        Servo bucket = hardwareMap.get(Servo.class, "bucket");
        bucket.setDirection(Servo.Direction.FORWARD);
        double bucketTarget = 0;


        // Initialize intake
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setPower(0);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        boolean intakeEnabled = false;
        boolean bPrevPressed = false;


        // Initialize stud and l1 → linear servo that acts as a pusher
        Servo stud = hardwareMap.get(Servo.class, "stud");
        Servo l1 = hardwareMap.get(Servo.class, "linear");
        stud.setDirection(Servo.Direction.FORWARD);
        l1.setDirection(Servo.Direction.FORWARD);
        stud.scaleRange(STUD_MIN, STUD_MAX);
        boolean studEnabled = false;
        boolean aPrevPressed = false;


        // Initialize plane launcher
        Servo plane = hardwareMap.get(Servo.class, "plane");
        plane.setDirection(Servo.Direction.FORWARD);
        
        // Initialize hang system
        DcMotor hang = hardwareMap.get(DcMotor.class, "hang");
        hang.setDirection(DcMotor.Direction.FORWARD);
        //hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setTargetPosition(0);
        //hang.setPower(1);
        //hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double hangTarget = 0;


        // Start
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Main loop
        while (opModeIsActive()) {


            // Drive logic
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x + lateral * STRAFE_YAW_BLEND;
            double leftFP = axial + lateral + yaw;
            double rightFP = axial - lateral - yaw;
            double leftBP = axial - lateral + yaw;
            double rightBP = axial + lateral - yaw;
            double max = Math.max(Math.abs(leftFP), Math.abs(rightFP));
            max = Math.max(max, Math.abs(leftBP));
            max = Math.max(max, Math.abs(rightBP));
            if (max > 1.0) {
                leftFP /= max;
                rightFP /= max;
                leftBP /= max;
                rightBP /= max;
            }
            frontLeftDrive.setPower(leftFP);
            frontRightDrive.setPower(rightFP);
            backLeftDrive.setPower(leftBP);
            backRightDrive.setPower(rightBP);
            
            // Arm logic
            double armInput = gamepad2.left_trigger - gamepad2.right_trigger;
            armTarget += armInput * ARM_MAX * DELTA_TIME / ARM_MIN_TIME;
            armTarget = Math.min(Math.max(0, armTarget), ARM_MAX);
            if (Math.abs(armInput) > MIN_INPUT_THRESHOLD) {
                arm.setTargetPosition((int)armTarget);
                
            // Bucket automation
                if (armTarget < 70) {
                    bucketTarget = BUCKET_FLOOR_POS;
                } else if (armTarget < 100) {
                    bucketTarget = 0;
                } else if (armTarget < 600) {
                    bucketTarget = BUCKET_ARM_M_1 * armTarget;
                } else {
                    bucketTarget = BUCKET_ARM_M_2 * armTarget + BUCKET_ARM_B_2;
                }
                bucketTarget = Math.min(Math.max(0, bucketTarget), 1);
                bucket.setPosition(bucketTarget);
            }


            /** Bucket logic
             * bucketTarget += -gamepad2.left_stick_y * DELTA_TIME / ARM_MIN_TIME; /
 * Assume bucket and arm take same amount of time
             * bucketTarget = Math.min(Math.max(0, bucketTarget), 1);
             * if (Math.abs(gamepad2.left_stick_y) > MIN_INPUT_THRESHOLD) {
             *  bucket.setPosition(bucketTarget);
             */
            
            // Intake logic
            if (!bPrevPressed && gamepad2.b) {
                intakeEnabled = !intakeEnabled;
            }
            bPrevPressed = gamepad2.b;
            if (gamepad2.left_bumper) {
                intake.setPower(-INTAKE_POWER);
            } else if (intakeEnabled) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0);
            }
            
            // Stud logic
            if (!aPrevPressed && gamepad2.a) {
                studEnabled = !studEnabled;
                if (studEnabled) {
                    stud.setPosition(1);
                } else {
                    stud.setPosition(0);
                }
            }
            aPrevPressed = gamepad2.a;


            // Plane logic
            if (gamepad1.x) {
                plane.setPosition(0);
             }
            if (gamepad1.a){
                plane.setPosition(1);
             }
            
            // Pixel Dragger Logic
            if (gamepad1.y) {
                l1.setPosition(0);
            }
            if (gamepad1.b) {
                l1.setPosition(1);
            }
            
            // Hang logic
            float hangInput = gamepad1.right_trigger - gamepad1.left_trigger;
            hang.setPower(hangInput);
            //hangTarget += hangInput * HANG_SPEED * DELTA_TIME;
            //hangTarget = Math.max(0, hangTarget);
            //if (Math.abs(hangInput) > MIN_INPUT_THRESHOLD) {
            //    hang.setTargetPosition((int)hangTarget);
            //}


            // Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("armPosition", arm.getCurrentPosition());
            telemetry.addData("armInput", armInput);
            telemetry.addData("armTarget", armTarget);
            telemetry.addData("bucketTarget", bucketTarget);
            telemetry.addData("bucket.getPosition()", bucket.getPosition());
            // telemetry.addData("bucket.MAX_POSITION", bucket.MAX_POSITION);
            // telemetry.addData("bucket.MIN_POSITION", bucket.MIN_POSITION);
            // telemetry.addData("bucket.getPortNumber()", bucket.getPortNumber());
            // telemetry.addData("intakeEnabled", intakeEnabled);
            // telemetry.addData("intake.getPower()", intake.getPower());
            // telemetry.addData("intake.getCurrentPosition()", intake.getCurrentPosition());
             telemetry.addData("studEnabled", studEnabled);
             telemetry.addData("stud.getPosition()", stud.getPosition());
             telemetry.addData("stud.MAX_POSITION", stud.MAX_POSITION);
             telemetry.addData("stud.MIN_POSITION", stud.MIN_POSITION);
            // telemetry.addData("plane.getPosition()", plane.getPosition());
            // telemetry.addData("plane.MAX_POSITION", plane.MAX_POSITION);
            // telemetry.addData("plane.MIN_POSITION", plane.MIN_POSITION);
            telemetry.addData("backRightDrive", backRightDrive.getPower());
            telemetry.addData("backLeftDrive", backLeftDrive.getPower());
            telemetry.addData("hangTarget", hangTarget);
            telemetry.update();
        }
    }
}