/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import java.util.TimeZone;
import org.mtr.core.generated.oba.AgencySchema;
import org.mtr.core.serializer.ReaderBase;

public final class Agency
extends AgencySchema {
    public Agency() {
        super("1", "My Agency", "https://github.com/jonafanho/Transport-Simulation-Core", TimeZone.getDefault().getID());
        this.lang = "en";
    }

    public Agency(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

