/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import org.mtr.core.generated.oba.FrequencySchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;

public final class Frequency
extends FrequencySchema
implements Utilities {
    public Frequency(long currentMillis) {
        super(0L, currentMillis + 86400000L, 8L);
    }

    public Frequency(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

