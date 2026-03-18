/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 */
package org.mtr.core.servlet;

import java.util.function.Consumer;
import javax.annotation.Nullable;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.serializer.WriterBase;

public final class QueueObject {
    public final String key;
    public final SerializedDataBase data;
    @Nullable
    private final Consumer<SerializedDataBase> callback;

    public <T extends SerializedDataBase> QueueObject(String key, SerializedDataBase data, @Nullable Consumer<T> callback, @Nullable Class<T> reaponseDataClass) {
        this.key = key;
        this.data = data;
        this.callback = callback == null || reaponseDataClass == null ? null : serializedDataBase -> {
            if (reaponseDataClass.isInstance(serializedDataBase)) {
                callback.accept(reaponseDataClass.cast(serializedDataBase));
            }
        };
    }

    public void runCallback(@Nullable SerializedDataBase data) {
        if (this.callback != null) {
            this.callback.accept(data == null ? new SerializedDataBase(){

                @Override
                public void updateData(ReaderBase readerBase) {
                }

                @Override
                public void serializeData(WriterBase writerBase) {
                }
            } : data);
        }
    }
}

