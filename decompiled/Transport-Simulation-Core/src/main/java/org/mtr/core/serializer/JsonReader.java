/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.com.google.gson.JsonElement
 *  org.mtr.libraries.com.google.gson.JsonObject
 *  org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectArrayMap
 */
package org.mtr.core.serializer;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.com.google.gson.JsonElement;
import org.mtr.libraries.com.google.gson.JsonObject;
import org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectArrayMap;

public final class JsonReader
extends ReaderBase {
    private final Object2ObjectArrayMap<String, JsonElement> map;

    public JsonReader(JsonElement value) {
        this.map = new Object2ObjectArrayMap();
        JsonReader.iterateMap(value, (arg_0, arg_1) -> this.map.put(arg_0, arg_1));
    }

    private JsonReader(Object2ObjectArrayMap<String, JsonElement> map) {
        this.map = map;
    }

    @Override
    public void unpackBoolean(String key, BooleanConsumer ifExists) {
        this.unpack(key, value -> ifExists.accept(JsonReader.getBoolean(value)));
    }

    @Override
    public boolean getBoolean(String key, boolean defaultValue) {
        return this.getOrDefault(key, defaultValue, JsonReader::getBoolean);
    }

    @Override
    public void iterateBooleanArray(String key, Runnable clearList, BooleanConsumer ifExists) {
        this.unpack(key, value -> JsonReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(JsonReader.getBoolean(arrayValue))));
    }

    @Override
    public void unpackInt(String key, IntConsumer ifExists) {
        this.unpack(key, value -> ifExists.accept(JsonReader.getInt(value)));
    }

    @Override
    public int getInt(String key, int defaultValue) {
        return this.getOrDefault(key, defaultValue, JsonReader::getInt);
    }

    @Override
    public void iterateIntArray(String key, Runnable clearList, IntConsumer ifExists) {
        this.unpack(key, value -> JsonReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(JsonReader.getInt(arrayValue))));
    }

    @Override
    public void unpackLong(String key, LongConsumer ifExists) {
        this.unpack(key, value -> ifExists.accept(JsonReader.getLong(value)));
    }

    @Override
    public long getLong(String key, long defaultValue) {
        return this.getOrDefault(key, defaultValue, JsonReader::getLong);
    }

    @Override
    public void iterateLongArray(String key, Runnable clearList, LongConsumer ifExists) {
        this.unpack(key, value -> JsonReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(JsonReader.getLong(arrayValue))));
    }

    @Override
    public void unpackDouble(String key, DoubleConsumer ifExists) {
        this.unpack(key, value -> ifExists.accept(JsonReader.getDouble(value)));
    }

    @Override
    public double getDouble(String key, double defaultValue) {
        return this.getOrDefault(key, defaultValue, JsonReader::getDouble);
    }

    @Override
    public void iterateDoubleArray(String key, Runnable clearList, DoubleConsumer ifExists) {
        this.unpack(key, value -> JsonReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(JsonReader.getDouble(arrayValue))));
    }

    @Override
    public void unpackString(String key, Consumer<String> ifExists) {
        this.unpack(key, value -> ifExists.accept(JsonReader.getString(value)));
    }

    @Override
    public String getString(String key, String defaultValue) {
        return this.getOrDefault(key, defaultValue, JsonReader::getString);
    }

    @Override
    public void iterateStringArray(String key, Runnable clearList, Consumer<String> ifExists) {
        this.unpack(key, value -> JsonReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(JsonReader.getString(arrayValue))));
    }

    @Override
    public void iterateReaderArray(String key, Runnable clearList, Consumer<ReaderBase> ifExists) {
        this.unpack(key, value -> JsonReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(new JsonReader((JsonElement)arrayValue))));
    }

    @Override
    public ReaderBase getChild(String key) {
        return this.getOrDefault(key, new JsonReader((JsonElement)new JsonObject()), JsonReader::new);
    }

    @Override
    public void unpackChild(String key, Consumer<ReaderBase> ifExists) {
        this.unpack(key, value -> ifExists.accept(new JsonReader((JsonElement)value)));
    }

    @Override
    public void merge(ReaderBase readerBase) {
        if (readerBase instanceof JsonReader) {
            this.map.putAll(((JsonReader)readerBase).map);
        }
    }

    private void unpack(String key, Consumer<JsonElement> consumer) {
        this.unpackValue(this.map.get(key), consumer);
    }

    private <T> T getOrDefault(String key, T defaultValue, Function<JsonElement, T> function) {
        return this.getValueOrDefault(this.map.get(key), defaultValue, function);
    }

    public static JsonReader parse(String string) {
        return new JsonReader((JsonElement)Utilities.parseJson(string));
    }

    private static boolean getBoolean(JsonElement value) {
        return value.getAsBoolean();
    }

    private static int getInt(JsonElement value) {
        return value.getAsInt();
    }

    private static long getLong(JsonElement value) {
        return value.getAsLong();
    }

    private static double getDouble(JsonElement value) {
        return value.getAsDouble();
    }

    private static String getString(JsonElement value) {
        return value.getAsString();
    }

    private static void iterateArray(JsonElement value, Runnable clearList, Consumer<JsonElement> consumer) {
        clearList.run();
        value.getAsJsonArray().forEach(arrayValue -> {
            try {
                consumer.accept((JsonElement)arrayValue);
            }
            catch (Exception exception) {
                // empty catch block
            }
        });
    }

    private static void iterateMap(JsonElement value, BiConsumer<String, JsonElement> consumer) {
        value.getAsJsonObject().asMap().forEach((mapKey, mapValue) -> {
            try {
                consumer.accept((String)mapKey, (JsonElement)mapValue);
            }
            catch (Exception exception) {
                // empty catch block
            }
        });
    }
}

