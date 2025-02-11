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

// TODO: test this function in runtime

/**
 * Provides simple, declarative way to implement <code>Sendable</code>.
 */
public interface SimpleSendable extends Sendable {

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
        String name();

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

        /**
         * Allowed ways to be accessed with NetworkTable. <code>Out</code> by default.
         *
         * @return the permission, see <code>AccessType</code>
         */
        AccessType access() default AccessType.Out;

    }

    /**
     * Title of the class' NetworkTable Entry.
     *
     * @return the name
     */
    String sendableTitle();

    /**
     * Optional initialization where complex entries may be added. Executed after all the <code>Properties</code> are
     * initialized.
     */
    default void optionalInit() {
    }

    @Override
    default void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(this.sendableTitle());
        final var childClassName = this.getClass().getName();
        Class<?> childClass;
        try {
            childClass = ClassLoader.getSystemClassLoader().loadClass(childClassName);
        }
        // impossible
        catch (ClassNotFoundException e) {
            throw new RuntimeException(e);
        }
        for (final var field : childClass.getFields()) {
            if (!field.isAnnotationPresent(Property.class)) continue;
            final var fieldType = field.getDeclaringClass().getName();
            if (fieldType.equals(String.class.getName())) this.initDoubleProperty(builder, field);
            else if (fieldType.equals(double.class.getName()) || fieldType.equals(Double.class.getName()))
                this.initStringProperty(builder, field);
            else if (fieldType.equals(boolean.class.getName()) || fieldType.equals(Boolean.class.getName()))
                this.initBooleanProperty(builder, field);
            else if (fieldType.equals(float.class.getName()) || fieldType.equals(Float.class.getName()))
                this.initFloatProperty(builder, field);
            else if (fieldType.equals(short.class.getName()) || fieldType.equals(Short.class.getName()) || fieldType.equals(int.class.getName()) || fieldType.equals(Integer.class.getName()) || fieldType.equals(long.class.getName()) || fieldType.equals(Long.class.getName()))
                this.initIntegerProperty(builder, field);
            else this.initObjectProperty(builder, field);
        }
        this.optionalInit();
    }

    private void initStringProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final Supplier<String> getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return field.get(this).toString();
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        final Consumer<String> setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, value);
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        builder.addStringProperty(property.name(), getter, setter);
    }

    private void initBooleanProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final BooleanSupplier getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return (boolean) field.get(this);
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        final BooleanConsumer setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, value);
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        builder.addBooleanProperty(property.name(), getter, setter);
    }

    private void initDoubleProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final DoubleSupplier getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return (double) field.get(this);
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        final DoubleConsumer setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, value);
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        builder.addDoubleProperty(property.name(), getter, setter);
    }

    private void initFloatProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final FloatSupplier getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return (float) field.get(this);
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        final FloatConsumer setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, value);
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        builder.addFloatProperty(property.name(), getter, setter);
    }

    private void initIntegerProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final LongSupplier getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return (long) field.get(this);
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        final LongConsumer setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, value);
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        builder.addIntegerProperty(property.name(), getter, setter);
    }

    private void initObjectProperty(SendableBuilder builder, Field field) {
        final var property = field.getAnnotation(Property.class);
        final Supplier<String> getter = property.access() == Property.AccessType.In ? null : () -> {
            try {
                return field.get(this).toString();
            }
            // impossible
            catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        };
        final Consumer<String> setter = property.access() == Property.AccessType.Out ? null : (value) -> {
            try {
                field.set(this, field.getDeclaringClass().getConstructor(String.class).newInstance(value));
            }
            // possible
            catch (IllegalAccessException | InvocationTargetException | InstantiationException |
                   NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        };
        builder.addStringProperty(property.name(), getter, setter);
    }

}
