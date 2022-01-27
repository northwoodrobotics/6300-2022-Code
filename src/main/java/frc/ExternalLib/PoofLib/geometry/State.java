package frc.ExternalLib.PoofLib.geometry;

import frc.ExternalLib.PoofLib.util.Interpolable;
import frc.ExternalLib.PoofLib.util.CSVWritable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
