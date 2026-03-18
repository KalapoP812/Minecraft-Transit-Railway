/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.map;

import org.mtr.core.generated.map.DirectionsResponseSchema;
import org.mtr.core.map.DirectionsConnection;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class DirectionsResponse
extends DirectionsResponseSchema {
    public DirectionsResponse(long totalRefreshGraphTime, long totalRefreshArrivalsTime, long totalPathFindingTime, long longestRefreshGraphTime, long longestRefreshArrivalsTime, long longestPathFindingTime) {
        super(totalRefreshGraphTime, totalRefreshArrivalsTime, totalPathFindingTime, longestRefreshGraphTime, longestRefreshArrivalsTime, longestPathFindingTime);
    }

    public DirectionsResponse(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public ObjectArrayList<DirectionsConnection> getDirectionsConnections() {
        return this.connections;
    }
}

