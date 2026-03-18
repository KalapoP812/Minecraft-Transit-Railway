/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.serializer;

import java.util.Collection;
import org.mtr.core.serializer.SerializedDataBase;

public abstract class WriterBase {
    public abstract void writeBoolean(String var1, boolean var2);

    public abstract void writeInt(String var1, int var2);

    public abstract void writeLong(String var1, long var2);

    public abstract void writeDouble(String var1, double var2);

    public abstract void writeString(String var1, String var2);

    public abstract Array writeArray(String var1);

    public abstract WriterBase writeChild(String var1);

    public final void writeDataset(Collection<? extends SerializedDataBase> dataSet, String key) {
        Array writerBaseArray = this.writeArray(key);
        dataSet.forEach(data -> data.serializeData(writerBaseArray.writeChild()));
    }

    public static abstract class Array {
        public abstract void writeBoolean(boolean var1);

        public abstract void writeInt(int var1);

        public abstract void writeLong(long var1);

        public abstract void writeDouble(double var1);

        public abstract void writeString(String var1);

        public abstract WriterBase writeChild();
    }
}

