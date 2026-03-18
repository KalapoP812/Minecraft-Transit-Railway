/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.operation;

import org.mtr.core.data.Rail;
import org.mtr.core.generated.operation.RailsRequestSchema;
import org.mtr.core.operation.RailsResponse;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;

public final class RailsRequest
extends RailsRequestSchema {
    public RailsRequest() {
    }

    public RailsRequest(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public RailsResponse query(Simulator simulator) {
        RailsResponse railsResponse = new RailsResponse();
        this.railIds.forEach(railId -> {
            Rail rail = (Rail)simulator.railIdMap.get(railId);
            if (rail != null) {
                railsResponse.add(rail);
            }
        });
        return railsResponse;
    }

    public RailsRequest addRailId(String railId) {
        this.railIds.add(railId);
        return this;
    }
}

