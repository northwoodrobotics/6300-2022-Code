package frc.ExternalLib.MadTownLib.vectors;

import java.util.function.Function;
import frc.ExternalLib.PoofLib.geometry.Translation2d;

public class GeneralVectorField extends VectorField {
  public GeneralVectorField(Function<Translation2d,Translation2d> field) {
    field_ = field;
  }
  protected Function<Translation2d,Translation2d> field_;
  public Translation2d getVector(Translation2d here) {
	  return field_.apply(here).normalize();
  }
}