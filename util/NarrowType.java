package frc.libzodiac.util;

public final class NarrowType {
    public static boolean isVoid(Class<?> type) {
        return type.getName().equals(void.class.getName());
    }

    public static boolean isBool(Class<?> type) {
        return type.getName().equals(boolean.class.getName()) ||
               type.getName().equals(Boolean.class.getName());
    }

    public static boolean isAnyInt(Class<?> type) {
        return isShort(type) || isInt(type) || isLong(type);
    }

    public static boolean isShort(Class<?> type) {
        return type.getName().equals(short.class.getName()) ||
               type.getName().equals(Short.class.getName());
    }

    public static boolean isInt(Class<?> type) {
        return type.getName().equals(int.class.getName()) ||
               type.getName().equals(Integer.class.getName());
    }

    public static boolean isLong(Class<?> type) {
        return type.getName().equals(long.class.getName()) ||
               type.getName().equals(Long.class.getName());
    }

    public static boolean isAnyFloat(Class<?> type) {
        return isFloat(type) || isDouble(type);
    }

    public static boolean isFloat(Class<?> type) {
        return type.getName().equals(float.class.getName()) ||
               type.getName().equals(Float.class.getName());
    }

    public static boolean isDouble(Class<?> type) {
        return type.getName().equals(double.class.getName()) ||
               type.getName().equals(Double.class.getName());
    }

    public static boolean isString(Class<?> type) {
        return type.getName().equals(String.class.getName());
    }
}
