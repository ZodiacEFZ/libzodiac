package frc.libzodiac.api;

import java.security.InvalidParameterException;

/**
 * A sensor device.
 */
public interface Zensor {

    /**
     * Get the specified value read by the sensor.
     * 
     * @param value the name of the value
     * @return the value read
     * @throws InvalidParameterException if the value specified does not exist
     */
    default double get(String value) throws InvalidParameterException {
        if (!value.equals(""))
            throw new InvalidParameterException();
        return this.get();
    }

    /**
     * Get the default value read by the sensor. For sensors that could read only
     * one value, this is where it should be. For sensors that could read more than
     * one values, if it is not "apparent" enough to return one of such values, an
     * exception should be thrown.
     * 
     * @return the value
     */
    double get();
}
