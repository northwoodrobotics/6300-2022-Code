//Created by Spectrum3847
package frc.ExternalLib.SpectrumLib.gamepads;

import edu.wpi.first.wpilibj.Joystick;
import frc.ExternalLib.SpectrumLib.gamepads.SpectrumXbox.XboxAxis;

public class Triggers {
	Joystick controller;

	public Triggers(Joystick controller) {
		this.controller = controller;
	}

	public double getLeft() {
		if (this.controller.isConnected()) {
			return this.controller.getRawAxis(XboxAxis.LEFT_TRIGGER.value);
		} else {
			return 0;
		}
	}

	public double getRight() {
		if (this.controller.isConnected()) {
			return this.controller.getRawAxis(XboxAxis.RIGHT_TRIGGER.value);
		} else {
			return 0;
		}
	}

	public double getTwist() {
		return -getLeft() + getRight();
	}
}
