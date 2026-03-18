/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import org.mtr.core.generated.oba.TripWithStopTimesSchema;
import org.mtr.core.serializer.ReaderBase;

public final class TripWithStopTimes
extends TripWithStopTimesSchema {
    public TripWithStopTimes(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

