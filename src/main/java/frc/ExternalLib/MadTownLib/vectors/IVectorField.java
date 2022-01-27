package frc.ExternalLib.MadTownLib.vectors;

import frc.ExternalLib.PoofLib.geometry.Translation2d;

public interface IVectorField {
	public abstract Translation2d getVector(Translation2d here);
}