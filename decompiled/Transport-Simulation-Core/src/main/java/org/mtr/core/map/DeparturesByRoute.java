/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 */
package org.mtr.core.map;

import org.mtr.core.generated.map.DeparturesByRouteSchema;
import org.mtr.core.map.DeparturesByDeviation;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;

public final class DeparturesByRoute
extends DeparturesByRouteSchema {
    DeparturesByRoute(String id, Long2ObjectAVLTreeMap<LongArrayList> departuresForRoute) {
        super(id);
        departuresForRoute.forEach((deviation, departuresForDeviation) -> this.departures.add(new DeparturesByDeviation((long)deviation, (LongArrayList)departuresForDeviation)));
    }

    public DeparturesByRoute(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

