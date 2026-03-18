/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList
 */
package org.mtr.core.map;

import org.mtr.core.generated.map.DeparturesByDeviationSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList;

public final class DeparturesByDeviation
extends DeparturesByDeviationSchema {
    public DeparturesByDeviation(long deviation, LongArrayList departuresForDeviation) {
        super(deviation);
        this.departures.addAll((LongList)departuresForDeviation);
    }

    public DeparturesByDeviation(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

