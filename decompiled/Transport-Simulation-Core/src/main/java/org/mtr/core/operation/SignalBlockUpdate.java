/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 */
package org.mtr.core.operation;

import org.mtr.core.data.Rail;
import org.mtr.core.generated.operation.SignalBlockUpdateSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;

public final class SignalBlockUpdate
extends SignalBlockUpdateSchema {
    public SignalBlockUpdate(Rail rail) {
        super(rail.getHexId());
        rail.iteratePreBlockedSignalColors(arg_0 -> ((LongArrayList)this.preBlockedSignalColors).add(arg_0));
        rail.iterateCurrentlyBlockedSignalColors(arg_0 -> ((LongArrayList)this.currentlyBlockedSignalColors).add(arg_0));
    }

    public SignalBlockUpdate(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public String getRailId() {
        return this.railId;
    }

    public LongArrayList getPreBlockedSignalColors() {
        return this.preBlockedSignalColors;
    }

    public LongArrayList getCurrentlyBlockedSignalColors() {
        return this.currentlyBlockedSignalColors;
    }
}

