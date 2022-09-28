//Created by Spectrum3847
package frc.ExternalLib.SpectrumLib.gamepads;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.ExternalLib.SpectrumLib.drivers.SpectrumDigitalInput;

public class DigitalInputButton extends Button {

	SpectrumDigitalInput input;

	public DigitalInputButton(SpectrumDigitalInput i) {
		input = i;
	}

	public boolean get() {
		return !input.get();
	}
}
