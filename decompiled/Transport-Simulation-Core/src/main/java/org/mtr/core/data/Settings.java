/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

import org.mtr.core.generated.data.SettingsSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;

public class Settings
extends SettingsSchema
implements Utilities {
    public Settings(long lastSimulationMillis) {
        super(lastSimulationMillis);
    }

    public Settings(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    @Override
    public String getHexId() {
        return Utilities.numberToPaddedHexString(0L);
    }

    @Override
    public boolean isValid() {
        return true;
    }

    public long getLastSimulationMillis() {
        long currentMillis = System.currentTimeMillis();
        return this.lastSimulationMillis == 0L ? currentMillis : currentMillis - (currentMillis - this.lastSimulationMillis) % 86400000L;
    }
}

