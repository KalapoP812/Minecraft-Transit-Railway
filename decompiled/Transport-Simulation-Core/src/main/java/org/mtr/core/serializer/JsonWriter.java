/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.com.google.gson.JsonArray
 *  org.mtr.libraries.com.google.gson.JsonElement
 *  org.mtr.libraries.com.google.gson.JsonObject
 */
package org.mtr.core.serializer;

import org.mtr.core.serializer.WriterBase;
import org.mtr.libraries.com.google.gson.JsonArray;
import org.mtr.libraries.com.google.gson.JsonElement;
import org.mtr.libraries.com.google.gson.JsonObject;

public final class JsonWriter
extends WriterBase {
    private final JsonObject jsonObject;

    public JsonWriter(JsonObject jsonObject) {
        this.jsonObject = jsonObject;
    }

    @Override
    public void writeBoolean(String key, boolean value) {
        this.jsonObject.addProperty(key, Boolean.valueOf(value));
    }

    @Override
    public void writeInt(String key, int value) {
        this.jsonObject.addProperty(key, (Number)value);
    }

    @Override
    public void writeLong(String key, long value) {
        this.jsonObject.addProperty(key, (Number)value);
    }

    @Override
    public void writeDouble(String key, double value) {
        this.jsonObject.addProperty(key, (Number)value);
    }

    @Override
    public void writeString(String key, String value) {
        this.jsonObject.addProperty(key, value);
    }

    @Override
    public WriterBase.Array writeArray(String key) {
        JsonArray jsonArray = new JsonArray();
        this.jsonObject.add(key, (JsonElement)jsonArray);
        return new JsonArrayWriter(jsonArray);
    }

    @Override
    public WriterBase writeChild(String key) {
        JsonObject childObject = new JsonObject();
        this.jsonObject.add(key, (JsonElement)childObject);
        return new JsonWriter(childObject);
    }

    private static final class JsonArrayWriter
    extends WriterBase.Array {
        private final JsonArray jsonArray;

        private JsonArrayWriter(JsonArray jsonArray) {
            this.jsonArray = jsonArray;
        }

        @Override
        public void writeBoolean(boolean value) {
            this.jsonArray.add(Boolean.valueOf(value));
        }

        @Override
        public void writeInt(int value) {
            this.jsonArray.add((Number)value);
        }

        @Override
        public void writeLong(long value) {
            this.jsonArray.add((Number)value);
        }

        @Override
        public void writeDouble(double value) {
            this.jsonArray.add((Number)value);
        }

        @Override
        public void writeString(String value) {
            this.jsonArray.add(value);
        }

        @Override
        public WriterBase writeChild() {
            JsonObject childObject = new JsonObject();
            this.jsonArray.add((JsonElement)childObject);
            return new JsonWriter(childObject);
        }
    }
}

