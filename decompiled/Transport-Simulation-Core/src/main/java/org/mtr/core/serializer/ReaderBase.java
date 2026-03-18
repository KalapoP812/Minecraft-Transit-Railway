/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongConsumer
 */
package org.mtr.core.serializer;

import java.util.function.Consumer;
import java.util.function.Function;
import javax.annotation.Nullable;
import org.mtr.core.Main;
import org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongConsumer;

public abstract class ReaderBase {
    public abstract void unpackBoolean(String var1, BooleanConsumer var2);

    public abstract boolean getBoolean(String var1, boolean var2);

    public abstract void iterateBooleanArray(String var1, Runnable var2, BooleanConsumer var3);

    public abstract void unpackInt(String var1, IntConsumer var2);

    public abstract int getInt(String var1, int var2);

    public abstract void iterateIntArray(String var1, Runnable var2, IntConsumer var3);

    public abstract void unpackLong(String var1, LongConsumer var2);

    public abstract long getLong(String var1, long var2);

    public abstract void iterateLongArray(String var1, Runnable var2, LongConsumer var3);

    public abstract void unpackDouble(String var1, DoubleConsumer var2);

    public abstract double getDouble(String var1, double var2);

    public abstract void iterateDoubleArray(String var1, Runnable var2, DoubleConsumer var3);

    public abstract void unpackString(String var1, Consumer<String> var2);

    public abstract String getString(String var1, String var2);

    public abstract void iterateStringArray(String var1, Runnable var2, Consumer<String> var3);

    public abstract void iterateReaderArray(String var1, Runnable var2, Consumer<ReaderBase> var3);

    public abstract ReaderBase getChild(String var1);

    public abstract void unpackChild(String var1, Consumer<ReaderBase> var2);

    public abstract void merge(ReaderBase var1);

    protected final <U> void unpackValue(@Nullable U value, Consumer<U> consumer) {
        if (value != null) {
            try {
                consumer.accept(value);
            }
            catch (Exception e) {
                Main.LOGGER.error("", (Throwable)e);
            }
        }
    }

    protected final <T, U> T getValueOrDefault(@Nullable U value, T defaultValue, Function<U, T> function) {
        if (value == null) {
            return defaultValue;
        }
        try {
            return function.apply(value);
        }
        catch (Exception ignored) {
            return defaultValue;
        }
    }
}

