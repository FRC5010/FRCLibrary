// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class Controller {

	private Joystick joystick;
	private boolean singleControllerMode;
	private JoystickButton A_BUTTON,
			B_BUTTON,
			X_BUTTON,
			Y_BUTTON,
			LEFT_BUMPER,
			RIGHT_BUMPER,
			START_BUTTON,
			BACK_BUTTON,
			LEFT_STICK_BUTT,
			RIGHT_STICK_BUTT;
	private POVButton UP, RIGHT, DOWN, LEFT;
	private Axis LEFT_X, LEFT_Y, L_TRIGGER, R_TRIGGER, RIGHT_X, RIGHT_Y;
	private Map<Integer, Axis> axisMap = new HashMap<>();

	public static class Axis {
		protected int port;
		protected Joystick joystick;
		protected Axis instance;

		public Axis(int port, Joystick joystick) {
			this.port = port;
			this.joystick = joystick;
		}

		public Axis() {
		}

		public double get() {
			return joystick.getRawAxis(port);
		}

		public Axis negate() {
			return new Negate(this);
		}

		public Axis negate(boolean invert) {
			return new Negate(this, invert);
		}

		public Axis cubed() {
			return new CurvePower(this);
		}

		public Axis curvePower(double power) {
			return new CurvePower(this, power);
		}

		public Axis deadzone(double deadzone) {
			return new Deadzone(this, deadzone);
		}

		public Axis limit(double limit) {
			return new HardLimit(this, limit);
		}

		public Axis rate(double limit) {
			return new ChangeRate(this, limit);
		}

		public Axis scale(double scale) {
			return new Scale(this, scale);
		}
	}

	private static class Negate extends Axis {
		boolean invert = true;

		public Negate(Axis axis) {
			instance = axis;
		}

		public Negate(Axis axis, boolean invert) {
			instance = axis;
			this.invert = invert;
		}

		public double get() {
			return invert ? -instance.get() : instance.get();
		}
	}

	private static class CurvePower extends Axis {
		double power = 3.0;

		public CurvePower(Axis axis) {
			instance = axis;
		}

		public CurvePower(Axis axis, double power) {
			instance = axis;
			this.power = power;
		}

		public double get() {
			return Math.pow(instance.get(), power);
		}
	}

	private static class Scale extends Axis {
		double scale;

		public Scale(Axis axis, double scale) {
			instance = axis;
			this.scale = scale;
		}

		public double get() {
			return scale * instance.get();
		}
	}

	private static class Deadzone extends Axis {
		double deadzone;

		public Deadzone(Axis axis, double deadzone) {
			instance = axis;
			this.deadzone = deadzone;
		}

		public double get() {
			double input = instance.get();
			if (input > -deadzone && input < deadzone) {
				return 0.0;
			}
			return input;
		}
	}

	public static class HardLimit extends Axis {
		double limit;

		public HardLimit(Axis axis, double limit) {
			instance = axis;
			this.limit = limit;
		}

		public double get() {
			double input = instance.get();
			if (input > limit) {
				return limit;
			}
			if (input < -limit) {
				return -limit;
			}

			return input;
		}
	}

	public static class ChangeRate extends Axis {
		SlewRateLimiter rateLimiter;

		public ChangeRate(Axis axis, double limit) {
			instance = axis;
			this.rateLimiter = new SlewRateLimiter(limit);
		}

		public double get() {
			double input = instance.get();
			return rateLimiter.calculate(input);
		}
	}

	private static enum ButtonNums {
		NO_BUTTON,
		A_BUTTON,
		B_BUTTON,
		X_BUTTON,
		Y_BUTTON,
		LEFT_BUMPER,
		RIGHT_BUMPER,
		START_BUTTON,
		BACK_BUTTON,
		LEFT_STICK_BUTT,
		RIGHT_STICK_BUTT;
	}

	public static enum JoystickPorts {
		ZERO,
		ONE,
		TWO,
		THREE
	}

	private static enum AxisNums {
		LEFT_X,
		LEFT_Y,
		L_TRIGGER,
		R_TRIGGER,
		RIGHT_X,
		RIGHT_Y
	}

	private static enum POVDirs {
		UP(0),
		RIGHT(90),
		DOWN(180),
		LEFT(270);

		public int direction;

		private POVDirs(int direction) {
			this.direction = direction;
		}
	}

	public Controller(int port) {
		joystick = new Joystick(port);
	}

	public void setRumble(double power) {
		joystick.setRumble(RumbleType.kBothRumble, power);
	}

	public boolean isPluggedIn() {
		return HIDType.kUnknown != joystick.getType();
	}

	public void setSingleControllerMode(boolean single) {
		singleControllerMode = single;
	}

	public boolean isSingleControllerMode() {
		return singleControllerMode;
	}

	public void setLeftYAxis(Axis yAxis) {
		LEFT_Y = yAxis;
	}

	public void setLeftXAxis(Axis xAxis) {
		LEFT_X = xAxis;
	}

	public void setRightYAxis(Axis yAxis) {
		RIGHT_Y = yAxis;
	}

	public void setRightXAxis(Axis xAxis) {
		RIGHT_X = xAxis;
	}

	public void setLeftTrigger(Axis leftTriggerAxis) {
		L_TRIGGER = leftTriggerAxis;
	}

	public void setRightTrigger(Axis rightTriggerAxis) {
		R_TRIGGER = rightTriggerAxis;
	}

	public Axis createLeftYAxis() {
		LEFT_Y = new Axis(AxisNums.LEFT_Y.ordinal(), joystick);
		return LEFT_Y;
	}

	public Axis createLeftXAxis() {
		LEFT_X = new Axis(AxisNums.LEFT_X.ordinal(), joystick);
		return LEFT_X;
	}

	public Axis createRightYAxis() {
		RIGHT_Y = new Axis(AxisNums.RIGHT_Y.ordinal(), joystick);
		return RIGHT_Y;
	}

	public Axis createRightXAxis() {
		RIGHT_X = new Axis(AxisNums.RIGHT_X.ordinal(), joystick);
		return RIGHT_X;
	}

	public Axis createLeftTrigger() {
		L_TRIGGER = new Axis(AxisNums.L_TRIGGER.ordinal(), joystick);
		return L_TRIGGER;
	}

	public Axis createRightTrigger() {
		R_TRIGGER = new Axis(AxisNums.R_TRIGGER.ordinal(), joystick);
		return R_TRIGGER;
	}

	public Axis createAxis(int channel) {
		Axis axis = new Axis(channel, joystick);
		axisMap.put(channel, axis);
		return axis;
	}

	public void setAxis(int channel, Axis axis) {
		axisMap.put(channel, axis);
	}

	public Axis getAxis(int channel) {
		return axisMap.get(channel);
	}

	public double getAxisValue(int channel) {
		Axis axis = axisMap.get(channel);
		if (null == axis) {
			return 0.0;
		}
		return axis.get();
	}

	public double getLeftYAxis() {
		return LEFT_Y.get();
	}

	public double getLeftXAxis() {
		return LEFT_X.get();
	}

	public double getRightYAxis() {
		return RIGHT_Y.get();
	}

	public double getRightXAxis() {
		return RIGHT_X.get();
	}

	public double getLeftTrigger() {
		return L_TRIGGER.get();
	}

	public double getRightTrigger() {
		return R_TRIGGER.get();
	}

	// A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, LEFT_BUMPER, RIGHT_BUMPER,
	// START_BUTTON, BACK_BUTTON, LEFT_STICK_BUTT, RIGHT_STICK_BUTT

	public JoystickButton createCustomButton(int buttonNum) {
		return new JoystickButton(joystick, buttonNum);
	}

	public JoystickButton createAButton() {
		A_BUTTON = new JoystickButton(joystick, ButtonNums.A_BUTTON.ordinal());
		return A_BUTTON;
	}

	public JoystickButton createBButton() {
		B_BUTTON = new JoystickButton(joystick, ButtonNums.B_BUTTON.ordinal());
		return B_BUTTON;
	}

	public JoystickButton createXButton() {
		X_BUTTON = new JoystickButton(joystick, ButtonNums.X_BUTTON.ordinal());
		return X_BUTTON;
	}

	public JoystickButton createYButton() {
		Y_BUTTON = new JoystickButton(joystick, ButtonNums.Y_BUTTON.ordinal());
		return Y_BUTTON;
	}

	public JoystickButton createLeftBumper() {
		LEFT_BUMPER = new JoystickButton(joystick, ButtonNums.LEFT_BUMPER.ordinal());
		return LEFT_BUMPER;
	}

	public JoystickButton createRightBumper() {
		RIGHT_BUMPER = new JoystickButton(joystick, ButtonNums.RIGHT_BUMPER.ordinal());
		return RIGHT_BUMPER;
	}

	public JoystickButton createStartButton() {
		START_BUTTON = new JoystickButton(joystick, ButtonNums.START_BUTTON.ordinal());
		return START_BUTTON;
	}

	public JoystickButton createBackButton() {
		BACK_BUTTON = new JoystickButton(joystick, ButtonNums.BACK_BUTTON.ordinal());
		return BACK_BUTTON;
	}

	public JoystickButton createLeftStickButton() {
		LEFT_STICK_BUTT = new JoystickButton(joystick, ButtonNums.LEFT_STICK_BUTT.ordinal());
		return LEFT_STICK_BUTT;
	}

	public JoystickButton createRightStickButton() {
		RIGHT_STICK_BUTT = new JoystickButton(joystick, ButtonNums.RIGHT_STICK_BUTT.ordinal());
		return RIGHT_STICK_BUTT;
	}

	public POVButton createUpPovButton() {
		UP = new POVButton(joystick, POVDirs.UP.direction);
		return UP;
	}

	public POVButton createDownPovButton() {
		DOWN = new POVButton(joystick, POVDirs.DOWN.direction);
		return DOWN;
	}

	public POVButton createLeftPovButton() {
		LEFT = new POVButton(joystick, POVDirs.LEFT.direction);
		return LEFT;
	}

	public POVButton createRightPovButton() {
		RIGHT = new POVButton(joystick, POVDirs.RIGHT.direction);
		return RIGHT;
	}
}
