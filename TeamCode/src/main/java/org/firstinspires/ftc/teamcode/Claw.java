package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {
    private DcMotor slideMotor;
    private DcMotor slideMotor2; // Second slide motor
    private Servo clawServo;
    private Servo wristServo;
    private Servo elbowServo;
    private TouchSensor limitSwitch;
    private ElapsedTime moveTimer;
    private int lastPosition;
    private static final double MOVE_TIMEOUT = 1.0; // 1 second timeout
    private static final int MIN_ENCODER_CHANGE = 5; // Minimum encoder change to consider movement
    
    // Servo positions
    private static final double CLAW_OPEN = 0.1;
    private static final double CLAW_CLOSED = 0.7;
    private static final double WRIST_UP = 0.2;
    private static final double WRIST_DOWN = 0.55;
    public static final double ELBOW_UP = 0.65;      // Fully raised position
    public static final double ELBOW_FORWARD = 0.35;  // Horizontal position
    public static final double ELBOW_DOWN = 0.0;     // Fully lowered position
    
    // Slide positions (in encoder ticks)
    private static final int SLIDE_GROUND = 0;
    private static final int SLIDE_LOW = 800;
    private static final int SLIDE_MEDIUM = 1350;
    private static final int SLIDE_HIGH = 3000;
    
    // Add minimum position for slide2
    private static final int SLIDE2_MIN_POSITION = 200;
    
    // Slide movement parameters
    private static final double SLIDE_POWER = 1;
    private static final double HOLDING_POWER = 0.1;  // Power to hold against gravity
    private static final int POSITION_TOLERANCE = 10;
    
    // Add state tracking for second slide
    private boolean isSlide2Active = false;
    private boolean isSlide2Syncing = false;
    private int slide2TargetPosition = 0;
    private static final int SYNC_TOLERANCE = 50; // Encoder ticks tolerance for considering slides synced
    
    // Add variables for gradual elbow movement
    private double currentElbowPosition = ELBOW_UP;  // Track current position
    private static final double ELBOW_MOVE_STEP = 0.02;  // How much to move per update
    
    public Claw(HardwareMap hardwareMap) {
        // Initialize limit switch first to fail fast if there's an issue
        try {
            limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
            if (limitSwitch == null) {
                throw new RuntimeException("Limit switch was found in hardware map but returned null");
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize limit switch. Check that:\n" +
                "1. Device name is exactly 'limit_switch'\n" +
                "2. Device is configured as REV Touch Sensor\n" +
                "3. Wire connections are secure\n" +
                "Error: " + e.getMessage());
        }
        
        // Initialize slide motors
        try {
            slideMotor = hardwareMap.get(DcMotor.class, "vertical_slide");
            slideMotor2 = hardwareMap.get(DcMotor.class, "vertical_slide2");
            
            // Configure main slide motor
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setDirection(DcMotor.Direction.REVERSE);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setPower(0);
            
            // Configure second slide motor and move to initial position
            slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor2.setDirection(DcMotor.Direction.FORWARD);
            slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor2.setTargetPosition(SLIDE2_MIN_POSITION);
            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor2.setPower(SLIDE_POWER);
            isSlide2Active = false;  // Ensure it starts inactive
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize slide motors: " + e.getMessage());
        }
        
        // Initialize servos
        try {
            clawServo = hardwareMap.get(Servo.class, "claw_servo");
            wristServo = hardwareMap.get(Servo.class, "wrist_servo");
            elbowServo = hardwareMap.get(Servo.class, "elbow_servo");
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize servos: " + e.getMessage());
        }
        
        // Initialize timer and position tracking
        moveTimer = new ElapsedTime();
        lastPosition = 0;
        
        // Initialize to safe starting position
        closeClaw();
        wristUp();
        elbowUp();  // Start with elbow in raised position
    }
    
    // Claw control methods
    public void openClaw() {
        clawServo.setPosition(CLAW_OPEN);
    }
    
    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED);
    }
    
    // Wrist control methods
    public void wristUp() {
        wristServo.setPosition(WRIST_UP);
    }
    
    public void wristDown() {
        wristServo.setPosition(WRIST_DOWN);
    }
    
    // Elbow control methods
    public void elbowUp() {
        currentElbowPosition = ELBOW_UP;
        elbowServo.setPosition(ELBOW_UP);
    }
    
    public void elbowForward() {
        currentElbowPosition = ELBOW_FORWARD;
        elbowServo.setPosition(ELBOW_FORWARD);
    }
    
    public void elbowDown() {
        currentElbowPosition = ELBOW_DOWN;
        elbowServo.setPosition(ELBOW_DOWN);
    }
    
    // Slide control methods
    public void moveToPosition(int targetPosition) {
        moveTimer.reset();
        lastPosition = slideMotor.getCurrentPosition();

        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setTargetPosition(targetPosition);
        slideMotor.setPower(SLIDE_POWER);

        // Only control second slide if it's active, and ensure minimum position
        if (isSlide2Active) {
            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int slide2Target = Math.max(targetPosition, SLIDE2_MIN_POSITION);
            slideMotor2.setTargetPosition(slide2Target);
            slideMotor2.setPower(SLIDE_POWER);
        }
    }
    
    public void moveToGround() {
        // Simply move to encoder position 0
        moveToPosition(SLIDE_GROUND);
    }
    
    public void moveToLow() {
        moveToPosition(SLIDE_LOW);
    }
    
    public void moveToMedium() {
        moveToPosition(SLIDE_MEDIUM);
    }
    
    public void moveToHigh() {
        moveToPosition(SLIDE_HIGH);
    }
    
    // Check if slide is at target position or stuck
    public boolean isAtTargetPosition() {
        int currentPosition = slideMotor.getCurrentPosition();
        int targetPosition = slideMotor.getTargetPosition();
        
        // If we're already at the target, return true
        if (Math.abs(currentPosition - targetPosition) < POSITION_TOLERANCE) {
            // Keep minimal power to hold position
            double holdingPower = (targetPosition > 100) ? HOLDING_POWER : 0;
            slideMotor.setPower(holdingPower);
            if (isSlide2Active) {
                slideMotor2.setPower(holdingPower);
            } else {
                slideMotor2.setPower(0);
            }
            return true;
        }
        
        // Check if we're stuck or moving too slowly
        if (moveTimer.seconds() > MOVE_TIMEOUT) {
            int positionChange = Math.abs(currentPosition - lastPosition);
            if (positionChange < MIN_ENCODER_CHANGE) {
                // Slide is stuck or moving too slowly
                stopSlide();
                return true;
            }
            // Reset timer and update last position for next check
            moveTimer.reset();
            lastPosition = currentPosition;
        }
        
        return false;
    }
    
    // Check if limit switch is pressed
    public boolean isLimitSwitchPressed() {
        return limitSwitch.isPressed();
    }
    
    // Method to check and handle limit switch during ground movement
    public void checkLimitSwitch() {
        if (isLimitSwitchPressed()) {
            // Stop both motors
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setPower(0);
            slideMotor2.setPower(0);
            
            // Reset encoder positions
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    // Emergency stop for both slides
    public void stopSlide() {
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (isSlide2Active) {
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double holdingPower = (slideMotor.getCurrentPosition() > 100) ? HOLDING_POWER : 0;
        slideMotor.setPower(holdingPower);
        if (isSlide2Active) {
            slideMotor2.setPower(holdingPower);
        }
    }
    
    // Set slide power with automatic holding power when near zero
    public void setSlidePower(double power) {
        if (Math.abs(power) < 0.01) {
            slideMotor.setPower(HOLDING_POWER);
            if (isSlide2Active) {
                slideMotor2.setPower(HOLDING_POWER);
            }
        } else {
            slideMotor.setPower(power);
            if (isSlide2Active) {
                slideMotor2.setPower(power);
            }
        }
    }
    
    // Get current slide position (using main slide as reference)
    public int getCurrentPosition() {
        return slideMotor.getCurrentPosition();
    }
    
    // Get current position of second slide
    public int getSecondSlidePosition() {
        return slideMotor2.getCurrentPosition();
    }
    
    // Getter methods for servo positions
    public double getClawPosition() {
        return clawServo.getPosition();
    }
    
    public double getWristPosition() {
        return wristServo.getPosition();
    }
    
    public double getElbowPosition() {
        return currentElbowPosition;
    }
    
    // Setter methods for manual control
    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }
    
    public void setWristPosition(double position) {
        wristServo.setPosition(position);
    }
    
    public void setElbowPosition(double position) {
        currentElbowPosition = position;
        elbowServo.setPosition(position);
    }

    // Add method to check if slide2 is at init position
    public boolean isSlide2AtInitPosition() {
        return Math.abs(slideMotor2.getCurrentPosition() - SLIDE2_MIN_POSITION) < POSITION_TOLERANCE;
    }

    // Modify startSlide2Sync to ensure proper transition from init position
    public void startSlide2Sync() {
        if (!isSlide2Active && !isSlide2Syncing) {
            isSlide2Syncing = true;
            slide2TargetPosition = Math.max(slideMotor.getCurrentPosition(), SLIDE2_MIN_POSITION);
            slideMotor2.setPower(0);  // Ensure power is zero before mode change
            slideMotor2.setTargetPosition(slide2TargetPosition);
            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor2.setPower(SLIDE_POWER);
        }
    }

    // Add method to maintain init position
    public void maintainSlide2InitPosition() {
        if (!isSlide2Active && !isSlide2Syncing) {
            if (slideMotor2.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            slideMotor2.setTargetPosition(SLIDE2_MIN_POSITION);
            slideMotor2.setPower(SLIDE_POWER);
        }
    }

    // Add method to check sync status
    public boolean isSlide2Synced() {
        if (isSlide2Syncing) {
            int currentPosition = slideMotor2.getCurrentPosition();
            if (Math.abs(currentPosition - slide2TargetPosition) < SYNC_TOLERANCE) {
                isSlide2Syncing = false;
                isSlide2Active = true;
                return true;
            }
        }
        return false;
    }

    // Add getter for slide2 state
    public boolean isSlide2Active() {
        return isSlide2Active;
    }

    public boolean isSlide2Syncing() {
        return isSlide2Syncing;
    }

    // Method to gradually move elbow to target position
    public void updateElbowPosition(double targetPosition) {
        if (Math.abs(currentElbowPosition - targetPosition) > ELBOW_MOVE_STEP) {
            if (currentElbowPosition < targetPosition) {
                currentElbowPosition += ELBOW_MOVE_STEP;
            } else {
                currentElbowPosition -= ELBOW_MOVE_STEP;
            }
            elbowServo.setPosition(currentElbowPosition);
        }
    }

    // Method to check if elbow has reached target
    public boolean isElbowAtTarget(double targetPosition) {
        return Math.abs(currentElbowPosition - targetPosition) <= ELBOW_MOVE_STEP;
    }
} 