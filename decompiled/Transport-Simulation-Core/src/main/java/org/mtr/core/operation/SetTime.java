/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.operation;

import org.mtr.core.generated.operation.SetTimeSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;

public final class SetTime
extends SetTimeSchema {
    public SetTime(long gameMillis, long millisPerDay, boolean isTimeMoving) {
        super(gameMillis, millisPerDay, isTimeMoving);
    }

    public SetTime(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void setGameTime(Simulator simulator) {
        simulator.setGameTime(this.gameMillis, this.millisPerDay, this.isTimeMoving);
    }
}

