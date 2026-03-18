/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.operation;

import org.mtr.core.data.Depot;
import org.mtr.core.generated.operation.DepotOperationByNameSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;

public final class DepotOperationByName
extends DepotOperationByNameSchema {
    public DepotOperationByName() {
    }

    public DepotOperationByName(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void setFilter(String filter) {
        this.filter = filter;
    }

    public void generate(Simulator simulator) {
        Depot.generateDepotsByName(simulator, this.filter);
    }

    public void clear(Simulator simulator) {
        Depot.clearDepotsByName(simulator, this.filter);
    }

    public void instantDeploy(Simulator simulator) {
        simulator.instantDeployDepotsByName(simulator, this.filter);
    }
}

