/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import org.mtr.core.generated.oba.TimeSchema;
import org.mtr.core.serializer.ReaderBase;

public final class Time
extends TimeSchema {
    public Time(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

