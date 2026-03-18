/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectArrayMap
 *  org.mtr.libraries.org.msgpack.core.MessageTypeException
 *  org.mtr.libraries.org.msgpack.core.MessageUnpacker
 *  org.mtr.libraries.org.msgpack.value.Value
 */
package org.mtr.core.serializer;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import org.mtr.core.Main;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.legacy.data.DataFixer;
import org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectArrayMap;
import org.mtr.libraries.org.msgpack.core.MessageTypeException;
import org.mtr.libraries.org.msgpack.core.MessageUnpacker;
import org.mtr.libraries.org.msgpack.value.Value;

public final class MessagePackReader
extends ReaderBase {
    private final Object2ObjectArrayMap<String, Value> map = new Object2ObjectArrayMap();

    public MessagePackReader() {
    }

    public MessagePackReader(MessageUnpacker messageUnpacker) throws MessageTypeException {
        try {
            int size = messageUnpacker.unpackMapHeader();
            for (int i = 0; i < size; ++i) {
                DataFixer.readerBaseConvertKey(messageUnpacker.unpackString(), (Value)messageUnpacker.unpackValue(), this.map);
            }
        }
        catch (MessageTypeException e) {
            throw e;
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
    }

    private MessagePackReader(Value value) {
        MessagePackReader.iterateMap(value, (mapKey, mapValue) -> DataFixer.readerBaseConvertKey(mapKey, mapValue, this.map));
    }

    @Override
    public void unpackBoolean(String key, BooleanConsumer ifExists) {
        this.unpack(key, value -> ifExists.accept(MessagePackReader.getBoolean(value)));
    }

    @Override
    public boolean getBoolean(String key, boolean defaultValue) {
        return this.getOrDefault(key, defaultValue, MessagePackReader::getBoolean);
    }

    @Override
    public void iterateBooleanArray(String key, Runnable clearList, BooleanConsumer ifExists) {
        this.unpack(key, value -> MessagePackReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(MessagePackReader.getBoolean(arrayValue))));
    }

    @Override
    public void unpackInt(String key, IntConsumer ifExists) {
        this.unpack(key, value -> ifExists.accept(MessagePackReader.getInt(value)));
    }

    @Override
    public int getInt(String key, int defaultValue) {
        return this.getOrDefault(key, defaultValue, MessagePackReader::getInt);
    }

    @Override
    public void iterateIntArray(String key, Runnable clearList, IntConsumer ifExists) {
        this.unpack(key, value -> MessagePackReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(MessagePackReader.getInt(arrayValue))));
    }

    @Override
    public void unpackLong(String key, LongConsumer ifExists) {
        this.unpack(key, value -> ifExists.accept(MessagePackReader.getLong(value)));
    }

    @Override
    public long getLong(String key, long defaultValue) {
        return this.getOrDefault(key, defaultValue, MessagePackReader::getLong);
    }

    @Override
    public void iterateLongArray(String key, Runnable clearList, LongConsumer ifExists) {
        this.unpack(key, value -> MessagePackReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(MessagePackReader.getLong(arrayValue))));
    }

    @Override
    public void unpackDouble(String key, DoubleConsumer ifExists) {
        this.unpack(key, value -> ifExists.accept(MessagePackReader.getDouble(value)));
    }

    @Override
    public double getDouble(String key, double defaultValue) {
        return this.getOrDefault(key, defaultValue, MessagePackReader::getDouble);
    }

    @Override
    public void iterateDoubleArray(String key, Runnable clearList, DoubleConsumer ifExists) {
        this.unpack(key, value -> MessagePackReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(MessagePackReader.getDouble(arrayValue))));
    }

    @Override
    public void unpackString(String key, Consumer<String> ifExists) {
        this.unpack(key, value -> ifExists.accept(MessagePackReader.getString(value)));
    }

    @Override
    public String getString(String key, String defaultValue) {
        return this.getOrDefault(key, defaultValue, MessagePackReader::getString);
    }

    @Override
    public void iterateStringArray(String key, Runnable clearList, Consumer<String> ifExists) {
        this.unpack(key, value -> MessagePackReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(MessagePackReader.getString(arrayValue))));
    }

    @Override
    public void iterateReaderArray(String key, Runnable clearList, Consumer<ReaderBase> ifExists) {
        this.unpack(key, value -> MessagePackReader.iterateArray(value, clearList, arrayValue -> ifExists.accept(new MessagePackReader((Value)arrayValue))));
    }

    @Override
    public ReaderBase getChild(String key) {
        return this.getOrDefault(key, new MessagePackReader(), MessagePackReader::new);
    }

    @Override
    public void unpackChild(String key, Consumer<ReaderBase> ifExists) {
        this.unpack(key, value -> ifExists.accept(new MessagePackReader((Value)value)));
    }

    @Override
    public void merge(ReaderBase readerBase) {
        if (readerBase instanceof MessagePackReader) {
            this.map.putAll(((MessagePackReader)readerBase).map);
        }
    }

    @Deprecated
    public void iterateMap(String key, BiConsumer<String, Value> consumer) {
        Value value = (Value)this.map.get(key);
        if (value != null) {
            MessagePackReader.iterateMap(value, consumer);
        }
    }

    private void unpack(String key, Consumer<Value> consumer) {
        this.unpackValue(this.map.get(key), consumer);
    }

    private <T> T getOrDefault(String key, T defaultValue, Function<Value, T> function) {
        return this.getValueOrDefault(this.map.get(key), defaultValue, function);
    }

    private static boolean getBoolean(Value value) {
        return value.asBooleanValue().getBoolean();
    }

    private static int getInt(Value value) {
        return value.asIntegerValue().asInt();
    }

    private static long getLong(Value value) {
        return value.asIntegerValue().asLong();
    }

    private static double getDouble(Value value) {
        return value.asFloatValue().toDouble();
    }

    private static String getString(Value value) {
        return value.asStringValue().asString();
    }

    private static void iterateArray(Value value, Runnable clearList, Consumer<Value> consumer) {
        clearList.run();
        value.asArrayValue().forEach(arrayValue -> {
            try {
                consumer.accept((Value)arrayValue);
            }
            catch (Exception exception) {
                // empty catch block
            }
        });
    }

    private static void iterateMap(Value value, BiConsumer<String, Value> consumer) {
        value.asMapValue().entrySet().forEach(entry -> {
            try {
                consumer.accept(MessagePackReader.getString((Value)entry.getKey()), (Value)entry.getValue());
            }
            catch (Exception exception) {
                // empty catch block
            }
        });
    }
}

