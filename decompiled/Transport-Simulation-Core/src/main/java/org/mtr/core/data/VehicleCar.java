/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

import org.mtr.core.generated.data.VehicleCarSchema;
import org.mtr.core.serializer.ReaderBase;

public final class VehicleCar
extends VehicleCarSchema {
    public final boolean hasOneBogie;

    public VehicleCar(String vehicleId, double length, double width, double bogie1Position, double bogie2Position, double couplingPadding1, double couplingPadding2) {
        super(vehicleId, length, width, bogie1Position, bogie2Position, couplingPadding1, couplingPadding2);
        this.hasOneBogie = this.bogie1Position == this.bogie2Position;
    }

    public VehicleCar(ReaderBase readerBase) {
        super(readerBase);
        this.hasOneBogie = this.bogie1Position == this.bogie2Position;
        this.updateData(readerBase);
    }

    public String getVehicleId() {
        return this.vehicleId;
    }

    public double getLength() {
        return this.length;
    }

    public double getWidth() {
        return this.width;
    }

    public double getBogie1Position() {
        return this.bogie1Position;
    }

    public double getBogie2Position() {
        return this.bogie2Position;
    }

    public double getTotalLength(boolean firstCar, boolean lastCar) {
        return this.getCouplingPadding1(firstCar) + this.length + this.getCouplingPadding2(lastCar);
    }

    double getCouplingPadding1(boolean firstCar) {
        return firstCar ? 0.0 : this.couplingPadding1;
    }

    double getCouplingPadding2(boolean lastCar) {
        return lastCar ? 0.0 : this.couplingPadding2;
    }
}

