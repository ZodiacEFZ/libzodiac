package frc.libzodiac.unused.util;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A utility class for performing lazy initialization.
 */
public final class Lazy<T> implements Supplier<T> {

    /**
     * The function that performs lazy initialization.
     */
    private final Supplier<T> supplier;
    /**
     * The lazy-initialized value, maybe uninitialized(null).
     */
    private Optional<T> value = Optional.empty();

    private Lazy(Supplier<T> supplier) {
        this.supplier = supplier;
    }

    /**
     * Create a lazy-initialized object with specified supplier.
     *
     * @param <T>      type of the lazy initialized value
     * @param supplier the function that provides the value
     * @return new
     */
    public static <T> Lazy<T> with(Supplier<T> supplier) {
        return new Lazy<>(supplier);
    }

    /**
     * Perform some action after the value is initialized.
     *
     * @param action the action to perform
     * @return a new `Lazy` instance
     * @apiNote The bahavior is undefined unless <code>this</code> has not been
     * evaluated yet.
     */
    public Lazy<T> then(Consumer<T> action) {
        return new Lazy<>(() -> {
            final var value = this.supplier.get();
            action.accept(value);
            return value;
        });
    }

    /**
     * Get the initialized value or perform initialization.
     */
    @Override
    public T get() {
        if (this.value.isEmpty()) {
            this.value = Optional.of(supplier.get());
        }
        return this.value.get();
    }

}
