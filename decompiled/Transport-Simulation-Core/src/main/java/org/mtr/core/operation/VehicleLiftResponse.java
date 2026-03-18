/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nonnull
 */
package org.mtr.core.operation;

import java.util.UUID;
import java.util.function.Consumer;
import java.util.function.LongConsumer;
import javax.annotation.Nonnull;
import org.mtr.core.data.Data;
import org.mtr.core.data.Lift;
import org.mtr.core.data.Rail;
import org.mtr.core.generated.operation.VehicleLiftResponseSchema;
import org.mtr.core.operation.SignalBlockUpdate;
import org.mtr.core.operation.VehicleUpdate;
import org.mtr.core.serializer.ReaderBase;

public final class VehicleLiftResponse
extends VehicleLiftResponseSchema {
    public final UUID uuid;
    private final Data data;

    public VehicleLiftResponse(UUID uuid, Data data) {
        super(uuid.toString());
        this.uuid = uuid;
        this.data = data;
    }

    public VehicleLiftResponse(ReaderBase readerBase, Data data) {
        super(readerBase);
        this.data = data;
        this.updateData(readerBase);
        this.uuid = UUID.fromString(this.clientId);
    }

    @Override
    @Nonnull
    protected Data liftsToUpdateDataParameter() {
        return this.data;
    }

    public void iterateVehiclesToUpdate(Consumer<VehicleUpdate> consumer) {
        this.vehiclesToUpdate.forEach(consumer);
    }

    public void iterateVehiclesToKeep(LongConsumer consumer) {
        this.vehiclesToKeep.forEach(consumer);
    }

    public void iterateLiftsToUpdate(Consumer<Lift> consumer) {
        this.liftsToUpdate.forEach(consumer);
    }

    public void iterateLiftsToKeep(LongConsumer consumer) {
        this.liftsToKeep.forEach(consumer);
    }

    public void iterateSignalBlockUpdates(Consumer<SignalBlockUpdate> consumer) {
        this.signalBlockUpdates.forEach(consumer);
    }

    public void addVehicleToUpdate(VehicleUpdate vehicleUpdate) {
        this.vehiclesToUpdate.add(vehicleUpdate);
    }

    public void addVehicleToKeep(long vehicleId) {
        this.vehiclesToKeep.add(vehicleId);
    }

    public void addLiftToUpdate(Lift lift) {
        this.liftsToUpdate.add(lift);
    }

    public void addLiftToKeep(long liftId) {
        this.liftsToKeep.add(liftId);
    }

    public void addSignalBlockUpdate(Rail rail) {
        this.signalBlockUpdates.add(new SignalBlockUpdate(rail));
    }
}

