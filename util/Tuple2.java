package frc.libzodiac.util;

public record Tuple2<T1, T2>(T1 x1, T2 x2) {
    @Override
    public T1 x1() {
        return x1;
    }

    @Override
    public T2 x2() {
        return x2;
    }
}
