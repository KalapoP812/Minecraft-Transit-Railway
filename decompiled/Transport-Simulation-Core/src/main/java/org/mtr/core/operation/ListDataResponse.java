/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nonnull
 */
package org.mtr.core.operation;

import java.util.Collection;
import javax.annotation.Nonnull;
import org.mtr.core.data.Data;
import org.mtr.core.generated.operation.ListDataResponseSchema;
import org.mtr.core.serializer.ReaderBase;

public final class ListDataResponse
extends ListDataResponseSchema {
    private final Data data;

    public ListDataResponse(Data data) {
        this.data = data;
    }

    public ListDataResponse(ReaderBase readerBase, Data data) {
        super(readerBase);
        this.data = data;
        this.updateData(readerBase);
    }

    @Override
    @Nonnull
    protected Data stationsDataParameter() {
        return this.data;
    }

    @Override
    @Nonnull
    protected Data platformsDataParameter() {
        return this.data;
    }

    @Override
    @Nonnull
    protected Data sidingsDataParameter() {
        return this.data;
    }

    @Override
    @Nonnull
    protected Data routesDataParameter() {
        return this.data;
    }

    @Override
    @Nonnull
    protected Data depotsDataParameter() {
        return this.data;
    }

    public ListDataResponse list() {
        this.stations.addAll(this.data.stations);
        this.platforms.addAll(this.data.platforms);
        this.sidings.addAll(this.data.sidings);
        this.routes.addAll(this.data.routes);
        this.depots.addAll(this.data.depots);
        return this;
    }

    public void write() {
        this.data.stations.clear();
        this.data.stations.addAll(this.stations);
        this.data.platforms.clear();
        this.data.platforms.addAll(this.platforms);
        this.data.sidings.clear();
        this.data.sidings.addAll(this.sidings);
        this.data.routes.clear();
        this.data.routes.addAll(this.routes);
        this.data.depots.clear();
        this.data.depots.addAll(this.depots);
        this.data.sync();
    }
}

