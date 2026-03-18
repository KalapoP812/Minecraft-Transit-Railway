/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import org.mtr.core.generated.oba.StopWithDistanceSchema;
import org.mtr.core.serializer.ReaderBase;

public final class StopWithDistance
extends StopWithDistanceSchema {
    public StopWithDistance(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

