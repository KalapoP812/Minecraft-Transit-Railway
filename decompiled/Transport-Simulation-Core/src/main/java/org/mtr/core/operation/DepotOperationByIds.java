/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.operation;

import org.mtr.core.data.Depot;
import org.mtr.core.generated.operation.DepotOperationByIdsSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class DepotOperationByIds
extends DepotOperationByIdsSchema {
    public DepotOperationByIds() {
    }

    public DepotOperationByIds(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void addDepotId(long depotId) {
        this.depotIds.add(depotId);
    }

    public void generate(Simulator simulator) {
        Depot.generateDepots(simulator, this.getDepots(simulator));
    }

    public void clear(Simulator simulator) {
        Depot.clearDepots(this.getDepots(simulator));
    }

    public void instantDeploy(Simulator simulator) {
        simulator.instantDeployDepots(this.getDepots(simulator));
    }

    private ObjectArrayList<Depot> getDepots(Simulator simulator) {
        ObjectArrayList depots = new ObjectArrayList();
        this.depotIds.forEach(depotId -> {
            Depot depot = (Depot)simulator.depotIdMap.get(depotId);
            if (depot != null) {
                depots.add(depot);
            }
        });
        return depots;
    }
}

