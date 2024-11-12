package frc.libzodiac;

import java.security.InvalidParameterException;

public interface Zensor {
    default double get(String value) throws InvalidParameterException {
        return this.get();
    }

    double get();
}
