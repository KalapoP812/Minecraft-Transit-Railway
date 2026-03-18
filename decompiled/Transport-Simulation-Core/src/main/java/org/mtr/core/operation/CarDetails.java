/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.operation;

import org.mtr.core.generated.operation.CarDetailsSchema;
import org.mtr.core.serializer.ReaderBase;

public final class CarDetails
extends CarDetailsSchema {
    public CarDetails(String vehicleId, double occupancy) {
        super(vehicleId, occupancy);
    }

    public CarDetails(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public String getVehicleId() {
        return this.vehicleId;
    }

    public double getOccupancy() {
        return this.occupancy;
    }
}

