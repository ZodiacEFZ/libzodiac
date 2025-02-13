package frc.libzodiac.util;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.lang.annotation.*;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.util.function.*;

/**
 * Provides simple, declarative way to implement <code>Sendable</code>.
 */
public interface SimpleSendable extends Sendable {
    @Override
    default void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(this.title());
        final var childClassName = this.getClass().getName();
        Class<?> childClass;
        try {
            childClass = ClassLoader.getSystemClassLoader().loadClass(childClassName);
        } catch (ClassNotFoundException ignored) {
            return;
        }
        for (final var field : childClass.getFields()) {
            if (!field.isAnnotationPresent(Property.class)) {
                continue;
            }
            final var fieldType = field.getType().getName();
            if (fieldType.equals(String.class.getName())) {
                this.initDoubleProperty(builder, field);
            } else if (fieldType.equals(double.class.getName()) || fieldType.equals(Double.class.getName())) {
                this.initStringProperty(builder, field);
            } else if (fieldType.equals(boolean.class.getName()) || fieldType.equals(Boolean.class.getName())) {
                this.initBooleanProperty(builder, field);
            } else if (fieldType.equals(float.class.getName()) || fieldType.equals(Float.class.getName())) {
                this.initFloatProperty(builder, field);
            } else if (fieldType.equals(short.class.getName()) || fieldType.equals(
                    Short.class.getName()) || fieldType.equals(int.class.getName()) || fieldType.equals(
                    Integer.class.getName()) || fieldType.equals(long.class.getName()) || fieldType.equals(
                    Long.class.getName())) {
                this.initIntegerProperty(builder, field);
            } else {
                this.initObjectProperty(builder, field);
            }
        }
        this.optSendableInit();
    }

    /**
     * Title of the class' NetworkTable Entry.
     *
     * @return the name
     */
    String title();

    private void initDoubleProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final DoubleSupplier getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return (double) field.get(this);
            } catch (IllegalAccessException ignored) {
            }
            return 0;
        };
        final DoubleConsumer setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, value);
            } catch (IllegalAccessException ignored) {
            }
        };
        final var name = property.name().isEmpty() ? field.getName() : property.name();
        builder.addDoubleProperty(name, getter, setter);
    }

    private void initStringProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final Supplier<String> getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return field.get(this).toString();
            } catch (IllegalAccessException ignored) {
            }
            return "";
        };
        final Consumer<String> setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, value);
            } catch (IllegalAccessException ignored) {
            }
        };
        final var name = property.name().isEmpty() ? field.getName() : property.name();
        builder.addStringProperty(name, getter, setter);
    }

    private void initBooleanProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final BooleanSupplier getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return (boolean) field.get(this);
            } catch (IllegalAccessException ignored) {
            }
            return false;
        };
        final BooleanConsumer setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, value);
            } catch (IllegalAccessException ignored) {
            }
        };
        final var name = property.name().isEmpty() ? field.getName() : property.name();
        builder.addBooleanProperty(name, getter, setter);
    }

    private void initFloatProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final FloatSupplier getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return (float) field.get(this);
            } catch (IllegalAccessException ignored) {
            }
            return 0;
        };
        final FloatConsumer setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, value);
            } catch (IllegalAccessException ignored) {
            }
        };
        final var name = property.name().isEmpty() ? field.getName() : property.name();
        builder.addFloatProperty(name, getter, setter);
    }

    private void initIntegerProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final LongSupplier getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return (long) field.get(this);
            } catch (IllegalAccessException ignored) {
            }
            return 0;
        };
        final LongConsumer setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, value);
            } catch (IllegalAccessException ignored) {
            }
        };
        final var name = property.name().isEmpty() ? field.getName() : property.name();
        builder.addIntegerProperty(name, getter, setter);
    }

    private void initObjectProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final Supplier<String> getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return field.get(this).toString();
            } catch (IllegalAccessException ignored) {
            }
            return "";
        };
        final Consumer<String> setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, field.getDeclaringClass().getConstructor(String.class).newInstance(value));
            } catch (IllegalAccessException | NoSuchMethodException | InstantiationException |
                     InvocationTargetException ignored) {
            }
        };
        final var name = property.name().isEmpty() ? field.getName() : property.name();
        builder.addStringProperty(name, getter, setter);
    }

    /**
     * Optional initialization where complex entries may be added. Executed after all the <code>Properties</code> are
     * initialized.
     */
    default void optSendableInit() {
    }

    /**
     * Annotates a field that is going to be sent to the NetworkTable.
     */
    @Target(ElementType.FIELD)
    @Retention(RetentionPolicy.RUNTIME)
    @Documented
    @interface Property {
        /**
         * Name of its NetworkTable entry.
         *
         * @return the name
         */
        String name() default "";

        /**
         * Allowed ways to be accessed with NetworkTable. <code>Out</code> by default.
         *
         * @return the permission, see <code>AccessType</code>
         */
        AccessType access() default AccessType.Out;

        enum AccessType {
            /**
             * The property is input-only.
             */
            In,
            /**
             * The property is output-only.
             */
            Out,
            /**
             * The property provides both input and output access for NetworkTable.
             */
            Both,
        }

    }
}