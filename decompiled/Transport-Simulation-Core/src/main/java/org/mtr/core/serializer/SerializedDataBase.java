/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.serializer;

import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.serializer.WriterBase;

public interface SerializedDataBase {
    public void updateData(ReaderBase var1);

    public void serializeData(WriterBase var1);

    default public void serializeFullData(WriterBase writerBase) {
        this.serializeData(writerBase);
    }
}

