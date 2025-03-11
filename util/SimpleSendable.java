package frc.libzodiac.util;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.annotation.*;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.function.*;

/**
 * Provides simple, declarative way to implement <code>Sendable</code>. <br/>
 * <b>Note</b> {@link SubsystemBase} implements {@link Sendable}, which conflicts with
 * this. Override <code>initSendable</code> manually and call <code>simpleSendableInit</code> method
 * provided as a workaround.
 */
public interface SimpleSendable extends Sendable {
    @Override
    default void initSendable(SendableBuilder builder) {
        this.simpleSendableInit(builder);
    }

    /**
     * Initialize this {@link Sendable} object, retrieving {@link SendableProperty} annotations on
     * fields.
     *
     * @param builder {@link SendableBuilder} passed on initialization
     */
    default void simpleSendableInit(SendableBuilder builder) {
        builder.setSmartDashboardType(this.title());
        final var childClass = this.getClass();
        final var entries = new Entries();
        for (final var method : childClass.getDeclaredMethods()) {
            if (method.isAnnotationPresent(SendableGetter.class)) {
                entries.addGetter(this, method);
            }
            if (method.isAnnotationPresent(SendableSetter.class)) {
                entries.addSetter(this, method);
            }
        }
        // use `getDeclaredFields` instead of `getFields` in order to attain private fields
        for (final var field : childClass.getDeclaredFields()) {
            if (field.isAnnotationPresent(SendableProperty.class)) {
                entries.addProperty(this, field);
            }
        }
        entries.init(builder);
    }

    /**
     * Title of the class' NetworkTable Entry.
     *
     * @return the name
     */
    default String title() {
        return this.getClass().getName();
    }

    @Target (ElementType.METHOD)
    @Retention (RetentionPolicy.RUNTIME)
    @Documented
    @interface SendableGetter {
        String name() default "";
    }

    @Target (ElementType.METHOD)
    @Retention (RetentionPolicy.RUNTIME)
    @Documented
    @interface SendableSetter {
        String name() default "";
    }

    /**
     * Annotates a field that is going to be sent to the NetworkTable.
     */
    @Target (ElementType.FIELD)
    @Retention (RetentionPolicy.RUNTIME)
    @Documented
    @interface SendableProperty {
        /**
         * Name of its NetworkTable entry. Empty by default. If the name is empty, field name is
         * automatically used as fallback.
         *
         * @return the name
         */
        String name() default "";

        /**
         * Allowed ways to be accessed with NetworkTable. <code>Out</code> by default.
         *
         * @return the permission, see {@link AccessType}
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

    final class Entries {
        final HashMap<String, Tuple2<BooleanSupplier, BooleanConsumer>> boolEntries = new HashMap<>();
        final HashMap<String, Tuple2<LongSupplier, LongConsumer>> intEntries = new HashMap<>();
        final HashMap<String, Tuple2<FloatSupplier, FloatConsumer>> floatEntries = new HashMap<>();
        final HashMap<String, Tuple2<DoubleSupplier, DoubleConsumer>> doubleEntries = new HashMap<>();
        final HashMap<String, Tuple2<Supplier<String>, Consumer<String>>> stringEntries = new HashMap<>();

        static <T> Supplier<T> wrap(Class<T> desiredType, Supplier<Object> supplier) {
            return () -> desiredType.cast(supplier.get());
        }

        void addGetter(Object obj, Method method) {
            // to bypass visibility control by java
            method.setAccessible(true);

            // getters shall not have parameters
            assert method.getParameterCount() == 0;

            final var returnType = method.getReturnType();
            final var getterInfo = method.getAnnotation(SendableGetter.class);
            final var name = getterInfo.name().isEmpty() ? method.getName() : getterInfo.name();
            final Supplier<Object> raw = () -> {
                try {
                    return method.invoke(obj);
                } catch (IllegalAccessException | InvocationTargetException e) {
                    throw new RuntimeException(e);
                }
            };
            if (NarrowType.isBool(returnType)) {
                this.add(name, wrap(Boolean.class, raw)::get);
            } else if (NarrowType.isAnyInt(returnType)) {
                this.add(name, wrap(Long.class, raw)::get);
            } else if (NarrowType.isFloat(returnType)) {
                this.add(name, wrap(Float.class, raw)::get);
            } else if (NarrowType.isDouble(returnType)) {
                this.add(name, wrap(Double.class, raw)::get);
            } else if (NarrowType.isString(returnType)) {
                //noinspection FunctionalExpressionCanBeFolded
                this.add(name, wrap(String.class, raw)::get);
            } else {
                this.add(name, () -> raw.get().toString());
            }
        }

        void add(String name, BooleanSupplier getter) throws NullPointerException {
            this.boolEntries.put(name, new Tuple2<>(getter, this.exists(name) ?
                                                                    this.boolEntries.get(name)
                                                                                    .x2() : null));
        }

        void add(String name, LongSupplier getter) throws NullPointerException {
            this.intEntries.put(name, new Tuple2<>(getter, this.exists(name) ?
                                                                   this.intEntries.get(name).x2() :
                                                                   null));
        }

        void add(String name, FloatSupplier getter) throws NullPointerException {
            this.floatEntries.put(name, new Tuple2<>(getter, this.exists(name) ?
                                                                     this.floatEntries.get(name)
                                                                                      .x2() :
                                                                     null));
        }

        void add(String name, DoubleSupplier getter) throws NullPointerException {
            this.doubleEntries.put(name, new Tuple2<>(getter, this.exists(name) ?
                                                                      this.doubleEntries.get(name)
                                                                                        .x2() :
                                                                      null));
        }

        void add(String name, Supplier<String> getter) throws NullPointerException {
            this.stringEntries.put(name, new Tuple2<>(getter, this.exists(name) ?
                                                                      this.stringEntries.get(name)
                                                                                        .x2() :
                                                                      null));
        }

        boolean exists(String name) {
            return intEntries.containsKey(name) || boolEntries.containsKey(name) ||
                   floatEntries.containsKey(name) || doubleEntries.containsKey(name) ||
                   stringEntries.containsKey(name);
        }

        void addSetter(Object obj, Method method) {
            // to bypass visibility control by java
            method.setAccessible(true);

            // setters shall have exactly one parameter
            assert method.getParameterCount() == 1;
            // setters shall return void
            assert NarrowType.isVoid(method.getReturnType());

            final var takeType = method.getParameterTypes()[0];
            final var setterInfo = method.getAnnotation(SendableSetter.class);
            final var name = setterInfo.name().isEmpty() ? method.getName() : setterInfo.name();
            final Consumer<Object> raw = x -> {
                try {
                    method.invoke(obj, x);
                } catch (IllegalAccessException | InvocationTargetException e) {
                    throw new RuntimeException(e);
                }
            };
            if (NarrowType.isBool(takeType)) {
                this.add(name, Entries.<Boolean> wrap(raw)::accept);
            } else if (NarrowType.isAnyInt(takeType)) {
                this.add(name, Entries.<Long> wrap(raw)::accept);
            } else if (NarrowType.isFloat(takeType)) {
                this.add(name, Entries.<Float> wrap(raw)::accept);
            } else if (NarrowType.isDouble(takeType)) {
                this.add(name, Entries.<Double> wrap(raw)::accept);
            } else if (NarrowType.isString(takeType)) {
                //noinspection RedundantTypeArguments
                this.add(name, Entries.<String> wrap(raw));
            } else {
                Consumer<String> setter = null;
                if (takeType.isAssignableFrom(String.class)) {
                    setter = raw::accept;
                } else {
                    try {
                        takeType.getConstructor(String.class);
                    } catch (NoSuchMethodException ignored) {
                        throw new UnsupportedOperationException("Type " + takeType.getName() +
                                                                " is neither assignable nor constructible from string");
                    }
                }
                this.add(name, setter);
            }
        }

        void add(String name, BooleanConsumer setter) throws NullPointerException {
            this.boolEntries.put(name, new Tuple2<>(
                    this.exists(name) ? this.boolEntries.get(name).x1() : null, setter));
        }

        void add(String name, LongConsumer setter) throws NullPointerException {
            this.intEntries.put(name, new Tuple2<>(
                    this.exists(name) ? this.intEntries.get(name).x1() : null, setter));
        }

        void add(String name, FloatConsumer setter) throws NullPointerException {
            this.floatEntries.put(name, new Tuple2<>(
                    this.exists(name) ? this.floatEntries.get(name).x1() : null, setter));
        }

        void add(String name, DoubleConsumer setter) throws NullPointerException {
            this.doubleEntries.put(name, new Tuple2<>(
                    this.exists(name) ? this.doubleEntries.get(name).x1() : null, setter));
        }

        void add(String name, Consumer<String> setter) throws NullPointerException {
            this.stringEntries.put(name, new Tuple2<>(
                    this.exists(name) ? this.stringEntries.get(name).x1() : null, setter));
        }

        static <T> Consumer<T> wrap(Consumer<Object> setter) {
            return setter::accept;
        }

        void addProperty(Object obj, Field field) {
            // to bypass visibility control by java
            field.setAccessible(true);

            final var fieldType = field.getType();
            final var propertyInfo = field.getAnnotation(SendableProperty.class);
            final var name = propertyInfo.name().isEmpty() ? field.getName() : propertyInfo.name();
            if (propertyInfo.access() != SendableProperty.AccessType.In) {
                final Supplier<Object> raw = () -> {
                    try {
                        return field.get(obj);
                    } catch (IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                };
                if (NarrowType.isBool(fieldType)) {
                    this.add(name, wrap(Boolean.class, raw)::get);
                } else if (NarrowType.isAnyInt(fieldType)) {
                    this.add(name, wrap(Long.class, raw)::get);
                } else if (NarrowType.isFloat(fieldType)) {
                    this.add(name, wrap(Float.class, raw)::get);
                } else if (NarrowType.isDouble(fieldType)) {
                    this.add(name, wrap(Double.class, raw)::get);
                } else if (NarrowType.isString(fieldType)) {
                    //noinspection FunctionalExpressionCanBeFolded
                    this.add(name, wrap(String.class, raw)::get);
                } else {
                    this.add(name, () -> raw.get().toString());
                }
            }
            if (propertyInfo.access() != SendableProperty.AccessType.Out) {
                final Consumer<Object> raw = x -> {
                    try {
                        field.set(obj, x);
                    } catch (IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                };
                if (NarrowType.isBool(fieldType)) {
                    this.add(name, Entries.<Boolean> wrap(raw)::accept);
                } else if (NarrowType.isAnyInt(fieldType)) {
                    this.add(name, Entries.<Long> wrap(raw)::accept);
                } else if (NarrowType.isFloat(fieldType)) {
                    this.add(name, Entries.<Float> wrap(raw)::accept);
                } else if (NarrowType.isDouble(fieldType)) {
                    this.add(name, Entries.<Double> wrap(raw)::accept);
                } else if (NarrowType.isString(fieldType)) {
                    //noinspection RedundantTypeArguments
                    this.add(name, Entries.<String> wrap(raw));
                } else {
                    Consumer<String> setter = null;
                    if (fieldType.isAssignableFrom(String.class)) {
                        setter = raw::accept;
                    } else {
                        try {
                            fieldType.getConstructor(String.class);
                        } catch (NoSuchMethodException ignored) {
                            throw new UnsupportedOperationException("Type " + fieldType.getName() +
                                                                    " is neither assignable nor constructible from string");
                        }
                    }
                    this.add(name, setter);
                }
            }
        }

        void init(SendableBuilder builder) {
            for (final var i : this.boolEntries.entrySet()) {
                builder.addBooleanProperty(i.getKey(), i.getValue().x1(), i.getValue().x2());
            }
            for (final var i : this.intEntries.entrySet()) {
                builder.addIntegerProperty(i.getKey(), i.getValue().x1(), i.getValue().x2());
            }
            for (final var i : this.floatEntries.entrySet()) {
                builder.addFloatProperty(i.getKey(), i.getValue().x1(), i.getValue().x2());
            }
            for (final var i : this.doubleEntries.entrySet()) {
                builder.addDoubleProperty(i.getKey(), i.getValue().x1(), i.getValue().x2());
            }
            for (final var i : this.stringEntries.entrySet()) {
                builder.addStringProperty(i.getKey(), i.getValue().x1(), i.getValue().x2());
            }
        }
    }
}