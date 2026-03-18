/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.operation;

import org.mtr.core.data.Vehicle;
import org.mtr.core.data.VehicleExtraData;
import org.mtr.core.generated.operation.VehicleUpdateSchema;
import org.mtr.core.serializer.ReaderBase;

public final class VehicleUpdate
extends VehicleUpdateSchema {
    public VehicleUpdate(Vehicle vehicle, VehicleExtraData data) {
        super(vehicle, data);
    }

    public VehicleUpdate(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public Vehicle getVehicle() {
        return this.vehicle;
    }

    public VehicleExtraData getVehicleExtraData() {
        return this.data;
    }
}

