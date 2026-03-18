/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.com.google.gson.GsonBuilder
 *  org.mtr.libraries.com.google.gson.JsonElement
 *  org.mtr.libraries.com.google.gson.JsonObject
 *  org.mtr.libraries.com.google.gson.JsonParser
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectLongImmutablePair
 */
package org.mtr.core.tool;

import java.util.Collection;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import javax.annotation.Nullable;
import org.mtr.core.Main;
import org.mtr.core.data.Position;
import org.mtr.core.serializer.JsonWriter;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.tool.ConditionalList;
import org.mtr.core.tool.Vector;
import org.mtr.libraries.com.google.gson.GsonBuilder;
import org.mtr.libraries.com.google.gson.JsonElement;
import org.mtr.libraries.com.google.gson.JsonObject;
import org.mtr.libraries.com.google.gson.JsonParser;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectLongImmutablePair;

public interface Utilities {
    public static final int HOURS_PER_DAY = 24;
    public static final int MILLIS_PER_SECOND = 1000;
    public static final int MILLIS_PER_MINUTE = 60000;
    public static final int MILLIS_PER_HOUR = 3600000;
    public static final int MILLIS_PER_DAY = 86400000;

    public static boolean isBetween(double value, double value1, double value2) {
        return Utilities.isBetween(value, value1, value2, 0.0);
    }

    public static boolean isBetween(double value, double value1, double value2, double padding) {
        return value >= Math.min(value1, value2) - padding && value <= Math.max(value1, value2) + padding;
    }

    public static boolean isBetween(Position position, Position position1, Position position2, double padding) {
        return Utilities.isBetween(position, position1.getX(), position1.getY(), position1.getZ(), position2.getX(), position2.getY(), position2.getZ(), padding);
    }

    public static boolean isBetween(Position position, Vector position1, Vector position2, double padding) {
        return Utilities.isBetween(position, position1.x(), position1.y(), position1.z(), position2.x(), position2.y(), position2.z(), padding);
    }

    public static boolean isBetween(Position position, double x1, double y1, double z1, double x2, double y2, double z2, double padding) {
        return Utilities.isBetween(position.getX(), x1, x2, padding) && Utilities.isBetween(position.getY(), y1, y2, padding) && Utilities.isBetween(position.getZ(), z1, z2, padding);
    }

    public static boolean isIntersecting(double value1, double value2, double value3, double value4) {
        return Utilities.isBetween(value3, value1, value2) || Utilities.isBetween(value4, value1, value2) || Utilities.isBetween(value1, value3, value4) || Utilities.isBetween(value2, value3, value4);
    }

    public static int clamp(int value, int min, int max) {
        return Math.min(max, Math.max(min, value));
    }

    public static long clamp(long value, long min, long max) {
        return Math.min(max, Math.max(min, value));
    }

    public static float clamp(float value, float min, float max) {
        return Math.min(max, Math.max(min, value));
    }

    public static double clamp(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    public static double round(double value, int decimalPlaces) {
        int factor = 1;
        for (int i = 0; i < decimalPlaces; ++i) {
            factor *= 10;
        }
        return (double)Math.round(value * (double)factor) / (double)factor;
    }

    public static double getAverage(double a, double b) {
        return (a + b) / 2.0;
    }

    public static String numberToPaddedHexString(long value) {
        return Utilities.numberToPaddedHexString(value, 16);
    }

    public static String numberToPaddedHexString(long value, int length) {
        return String.format("%" + length + "s", Long.toHexString(value)).replace(' ', '0').toUpperCase(Locale.ENGLISH);
    }

    public static String concat(Object ... objects) {
        StringBuilder stringBuilder = new StringBuilder();
        for (Object object : objects) {
            stringBuilder.append(object);
        }
        return stringBuilder.toString();
    }

    public static String formatName(String text) {
        return text.split("\\|\\|")[0].replace("|", " ");
    }

    public static JsonObject parseJson(String data) {
        try {
            return JsonParser.parseString((String)data).getAsJsonObject();
        }
        catch (Exception ignored) {
            return new JsonObject();
        }
    }

    public static String prettyPrint(String string) {
        return Utilities.prettyPrint((JsonElement)Utilities.parseJson(string));
    }

    public static String prettyPrint(JsonElement jsonElement) {
        return new GsonBuilder().setPrettyPrinting().create().toJson(jsonElement);
    }

    public static double kilometersPerHourToMetersPerMillisecond(double speedKilometersPerHour) {
        return speedKilometersPerHour / 3600.0;
    }

    public static <T, U extends List<T>> T getElement(U collection, int index) {
        return Utilities.getElement(collection, index, null);
    }

    public static <T, U extends List<T>> T getElement(@Nullable U collection, int index, @Nullable T defaultValue) {
        Object result = collection == null || index >= collection.size() || index < -collection.size() ? null : collection.get((index < 0 ? collection.size() : 0) + index);
        return (T)(result == null ? defaultValue : result);
    }

    public static <T, U extends List<T>> void setElement(@Nullable U collection, int index, T value) {
        if (collection != null && index < collection.size() && index >= -collection.size()) {
            collection.set((index < 0 ? collection.size() : 0) + index, value);
        }
    }

    @Nullable
    public static <T, U extends List<T>> T removeElement(@Nullable U collection, int index) {
        if (collection == null || index >= collection.size() || index < -collection.size()) {
            return null;
        }
        return collection.remove((index < 0 ? collection.size() : 0) + index);
    }

    public static <T extends ConditionalList> int getIndexFromConditionalList(List<T> list, double value) {
        if (list.isEmpty()) {
            return -1;
        }
        int listSize = list.size();
        int index = listSize / 2;
        int lowIndex = -1;
        int highIndex = listSize;
        while (true) {
            if (((ConditionalList)list.get(index)).matchesCondition(value)) {
                lowIndex = index;
            } else {
                highIndex = index;
            }
            if (lowIndex + 1 == highIndex) {
                return lowIndex < 0 ? -1 : lowIndex;
            }
            index = Utilities.clamp((lowIndex + highIndex) / 2, 0, listSize - 1);
        }
    }

    public static <T extends SerializedDataBase> JsonObject getJsonObjectFromData(T data) {
        JsonObject jsonObject = new JsonObject();
        data.serializeData(new JsonWriter(jsonObject));
        return jsonObject;
    }

    public static long circularDifference(long value1, long value2, long totalDegrees) {
        long tempValue1 = value1;
        long halfTotalDegrees = totalDegrees / 2L;
        if (tempValue1 - halfTotalDegrees > value2 || tempValue1 + halfTotalDegrees <= value2) {
            tempValue1 -= (tempValue1 - halfTotalDegrees - value2) / totalDegrees * totalDegrees;
        }
        while (tempValue1 - halfTotalDegrees > value2) {
            tempValue1 -= totalDegrees;
        }
        while (tempValue1 + halfTotalDegrees <= value2) {
            tempValue1 += totalDegrees;
        }
        return tempValue1 - value2;
    }

    public static int compare(long value1, long value2, IntSupplier ifZero) {
        int result = Long.compare(value1, value2);
        return result == 0 ? ifZero.getAsInt() : result;
    }

    public static int compare(String value1, String value2, IntSupplier ifZero) {
        try {
            return Utilities.compare(Long.parseLong(value1), Long.parseLong(value2), ifZero);
        }
        catch (Exception ignored) {
            int result = value1.compareTo(value2);
            return result == 0 ? ifZero.getAsInt() : result;
        }
    }

    public static <T> boolean sameItems(Collection<T> collection1, Collection<T> collection2) {
        return collection1.containsAll(collection2) && collection2.containsAll(collection1);
    }

    public static <T> T loopUntilTimeout(Supplier<T> action, long timeoutMillis) {
        long startMillis = System.currentTimeMillis();
        while (System.currentTimeMillis() - startMillis < timeoutMillis) {
            T result = action.get();
            if (result == null) continue;
            return result;
        }
        return null;
    }

    public static long measureDuration(Runnable action) {
        long startMillis = System.currentTimeMillis();
        action.run();
        return System.currentTimeMillis() - startMillis;
    }

    public static <T> ObjectLongImmutablePair<T> measureDuration(Supplier<T> action) {
        long startMillis = System.currentTimeMillis();
        return new ObjectLongImmutablePair(action.get(), System.currentTimeMillis() - startMillis);
    }

    public static void awaitTermination(ExecutorService executorService) {
        try {
            while (!executorService.awaitTermination(5L, TimeUnit.MINUTES)) {
                Main.LOGGER.warn("Termination failed, retrying...");
            }
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
    }
}

