package frc.ExternalLib.JackInTheBotLib.util;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}
