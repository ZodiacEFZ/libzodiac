package frc.libzodiac.unused.util;

import java.util.function.Function;

public class Result<T, E> {

    private final T ok_value;

    private final E err_value;

    private Result(T ok_value, E err_value) {
        this.ok_value = ok_value;
        this.err_value = err_value;
    }

    public T unwrap() {
        if (this.ok_value == null) {
            throw new RuntimeException("`unwrap` on an `Err` value");
        }
        return this.ok_value;
    }

    public E unwrap_err() {
        if (this.err_value == null) {
            throw new RuntimeException("`unwrap_err` on an `Ok` value");
        }
        return this.err_value;
    }

    public <U> Result<U, E> map(Function<T, U> mapping) {
        return switch (this.status()) {
            case Ok -> Result.ok(mapping.apply(this.ok_value));
            case Err -> Result.err(this.err_value);
        };
    }

    public Status status() {
        return this.ok_value != null ? Status.Ok : Status.Err;
    }

    public static <T, E> Result<T, E> ok(T value) {
        if (value == null) {
            throw new RuntimeException("value in a `Result` could not be null");
        }
        return new Result<>(value, null);
    }

    public static <T, E> Result<T, E> err(E value) {
        if (value == null) {
            throw new RuntimeException("value in a `Result` could not be null");
        }
        return new Result<>(null, value);
    }

    public <U> Result<T, U> map_err(Function<E, U> mapping) {
        return switch (this.status()) {
            case Ok -> Result.ok(this.ok_value);
            case Err -> Result.err(mapping.apply(this.err_value));
        };
    }

    public enum Status {
        Ok,
        Err,
    }

}
