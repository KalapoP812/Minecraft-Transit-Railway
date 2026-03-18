/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import org.mtr.core.generated.oba.ScheduleForStopSchema;
import org.mtr.core.serializer.ReaderBase;

public final class ScheduleForStop
extends ScheduleForStopSchema {
    public ScheduleForStop(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

