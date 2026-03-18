/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.operation;

import javax.annotation.Nullable;
import org.mtr.core.data.Siding;
import org.mtr.core.data.VehicleRidingEntity;
import org.mtr.core.generated.operation.UpdateVehicleRidingEntitiesSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class UpdateVehicleRidingEntities
extends UpdateVehicleRidingEntitiesSchema {
    public UpdateVehicleRidingEntities(long sidingId, long vehicleId) {
        super(sidingId, vehicleId);
    }

    public UpdateVehicleRidingEntities(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void add(VehicleRidingEntity vehicleRidingEntity) {
        this.ridingEntities.add(vehicleRidingEntity);
    }

    @Nullable
    public UpdateVehicleRidingEntities update(Simulator simulator) {
        Siding siding = (Siding)simulator.sidingIdMap.get(this.sidingId);
        if (siding == null) {
            return null;
        }
        siding.updateVehicleRidingEntities(this.vehicleId, (ObjectArrayList<VehicleRidingEntity>)this.ridingEntities);
        return this;
    }
}

