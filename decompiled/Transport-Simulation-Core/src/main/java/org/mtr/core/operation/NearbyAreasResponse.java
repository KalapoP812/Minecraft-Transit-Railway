/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nonnull
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 */
package org.mtr.core.operation;

import javax.annotation.Nonnull;
import org.mtr.core.data.AreaBase;
import org.mtr.core.data.Data;
import org.mtr.core.data.Depot;
import org.mtr.core.data.SavedRailBase;
import org.mtr.core.data.Station;
import org.mtr.core.generated.operation.NearbyAreasResponseSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;

public final class NearbyAreasResponse
extends NearbyAreasResponseSchema {
    private final Data data;

    NearbyAreasResponse(Data data) {
        this.data = data;
    }

    public NearbyAreasResponse(ReaderBase readerBase, Data data) {
        super(readerBase);
        this.data = data;
        this.updateData(readerBase);
    }

    @Override
    @Nonnull
    protected Data depotsDataParameter() {
        return this.data;
    }

    @Override
    @Nonnull
    protected Data stationsDataParameter() {
        return this.data;
    }

    public ObjectImmutableList<Station> getStations() {
        return new ObjectImmutableList((ObjectList)this.stations);
    }

    public ObjectImmutableList<Depot> getDepots() {
        return new ObjectImmutableList((ObjectList)this.depots);
    }

    <T extends AreaBase<T, U>, U extends SavedRailBase<U, T>> void add(T area) {
        if (area instanceof Station) {
            this.stations.add(((Station)area));
        } else if (area instanceof Depot) {
            this.depots.add(((Depot)area));
        }
    }
}

