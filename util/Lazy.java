package frc.libzodiac.util;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * A utility class for performing lazy initialization.
 */
public final class Lazy<T> implements Supplier<T> {

    /**
     * The lazy-initialized value, maybe uninitialized(null).
     */
    private Optional<T> value = Optional.empty();

    /**
     * The function that performs lazy initialization.
     */
    private final Supplier<T> supplier;

    /**
     * Create a lazy-initialized object with specified supplier.
     * 
     * @param supplier the function that provides the value
     */
    public Lazy(Supplier<T> supplier) {
        this.supplier = supplier;
    }

    /**
     * Get the initialized value or perform initialization.
     */
    @Override
    public T get() {
        if (this.value.isEmpty())
            this.value = Optional.of(supplier.get());
        return this.value.get();
    }

}
