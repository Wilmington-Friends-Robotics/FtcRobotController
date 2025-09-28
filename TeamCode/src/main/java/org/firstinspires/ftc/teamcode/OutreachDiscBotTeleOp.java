package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "OutreachDiscBotTeleOp", group = "TeleOp")
public class OutreachDiscBotTeleOp extends OpMode {
	// Two motors per side driving a belt that powers three wheels per side
	private DcMotorEx leftMotorB;
	private DcMotorEx rightMotorA;
	private DcMotorEx rightMotorB;
	// Flywheel motor (toggles with A)
	private DcMotorEx flywheel;

	// Drive config
	private boolean halfPowerMode = false;
	private boolean xButtonWasPressed = false;
	// Flywheel toggle state
	private boolean flywheelOn = false;
	private boolean aButtonWasPressed = false;
	// Active braking state for flywheel
	private boolean flywheelBraking = false;
	private final ElapsedTime flywheelBrakeTimer = new ElapsedTime();
	private static final double FLYWHEEL_BRAKE_TIME_S = 1; // duration of reverse burst
	private static final double FLYWHEEL_BRAKE_POWER = -1;  // reverse power (adjust as needed)

	@Override
	public void init() {
		// Map motors (rename to match your configuration names in the RC app)
		leftMotorB = hardwareMap.get(DcMotorEx.class, "left_motor_b");
		rightMotorA = hardwareMap.get(DcMotorEx.class, "right_motor_a");
		rightMotorB = hardwareMap.get(DcMotorEx.class, "right_motor_b");
		flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

		// Recommended motor setup for tank drive
		leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		leftMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// Flywheel setup
		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		flywheel.setDirection(DcMotor.Direction.FORWARD); // Positive power = clockwise (adjust if needed)
		flywheel.setPower(0);

		// Reverse one side so forward power makes robot move forward
		leftMotorB.setDirection(DcMotor.Direction.FORWARD);
		rightMotorA.setDirection(DcMotor.Direction.REVERSE);
		rightMotorB.setDirection(DcMotor.Direction.REVERSE);

		telemetry.addLine("OutreachDiscBotTeleOp initialized");
		telemetry.addLine("Controls: Left stick Y = forward/back, Right stick X = turn");
		telemetry.addLine("Press X to toggle Half Power mode");
		telemetry.addLine("Press A to toggle flywheel ON/OFF");
		telemetry.addLine("Note: Rename motor names in code if needed: left_motor_a/b, right_motor_a/b, flywheel");
		telemetry.update();
	}

	@Override
	public void loop() {
		// Toggle half power with X
		if (gamepad1.x) {
			if (!xButtonWasPressed) {
				halfPowerMode = !halfPowerMode;
				xButtonWasPressed = true;
			}
		} else {
			xButtonWasPressed = false;
		}

		// Toggle flywheel with A (non-blocking active braking on OFF)
		if (gamepad1.a) {
			if (!aButtonWasPressed) {
				flywheelOn = !flywheelOn;
				if (flywheelOn) {
					flywheelBraking = false;
					flywheel.setPower(1.0);
				} else {
					// start active brake burst
					flywheelBraking = true;
					flywheelBrakeTimer.reset();
					flywheel.setPower(FLYWHEEL_BRAKE_POWER);
				}
				aButtonWasPressed = true;
			}
		} else {
			aButtonWasPressed = false;
		}

		// Finish flywheel braking burst
		if (flywheelBraking) {
			if (flywheelBrakeTimer.seconds() >= FLYWHEEL_BRAKE_TIME_S) {
				flywheel.setPower(0.0);
				flywheelBraking = false;
			}
		}

		// Arcade drive: forward/back on left stick Y, turn on right stick X
		double forward = -gamepad1.left_stick_y; // invert to make up = forward
		double turn = gamepad1.right_stick_x;

		// Deadband
		forward = applyDeadband(forward, 0.05);
		turn = applyDeadband(turn, 0.05);

		// Optional shaping for finer control
		forward = shape(forward);
		turn = shape(turn);

		double leftPower = forward + turn;
		double rightPower = forward - turn;

		// Normalize to keep within [-1, 1]
		double maxMag = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
		leftPower /= maxMag;
		rightPower /= maxMag;

		if (halfPowerMode) {
			leftPower *= 0.5;
			rightPower *= 0.5;
		}

		setLeftPower(leftPower);
		setRightPower(rightPower);

		telemetry.addData("Mode", halfPowerMode ? "Half Power" : "Full Power");
		telemetry.addData("Left Power", "%.2f", leftPower);
		telemetry.addData("Right Power", "%.2f", rightPower);
		telemetry.addData("Flywheel", flywheelOn ? (flywheelBraking ? "ON->BRAKING" : "ON (clockwise)") : (flywheelBraking ? "BRAKING" : "OFF"));
		telemetry.update();
	}

	private void setLeftPower(double power) {
		leftMotorB.setPower(power);
	}

	private void setRightPower(double power) {
		rightMotorA.setPower(power);
		rightMotorB.setPower(power);
	}

	private double applyDeadband(double value, double threshold) {
		return Math.abs(value) < threshold ? 0.0 : value;
	}

	private double shape(double input) {
		// Cubic shaping keeps sign and provides finer low-speed control
		return input * input * input;
	}
} 