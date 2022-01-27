package frc.ExternalLib.MadTownLib.vectors;

import java.util.function.Function;

import frc.ExternalLib.PoofLib.geometry.Translation2d;

public interface ISurface {
	public abstract Function<Translation2d,Double> f();
	public abstract Function<Translation2d,Double> dfdx();
	public abstract Function<Translation2d,Double> dfdy();
}