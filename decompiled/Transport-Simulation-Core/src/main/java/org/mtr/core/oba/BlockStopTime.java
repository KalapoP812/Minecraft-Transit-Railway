/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import org.mtr.core.generated.oba.BlockStopTimeSchema;
import org.mtr.core.serializer.ReaderBase;

public final class BlockStopTime
extends BlockStopTimeSchema {
    public BlockStopTime(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

