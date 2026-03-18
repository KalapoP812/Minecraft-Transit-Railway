/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

import java.util.UUID;
import org.mtr.core.generated.data.VehicleRidingEntitySchema;
import org.mtr.core.serializer.ReaderBase;

public class VehicleRidingEntity
extends VehicleRidingEntitySchema {
    public final UUID uuid;

    public VehicleRidingEntity(UUID uuid, long ridingCar, double x, double y, double z, boolean isOnGangway, boolean isDriver, boolean manualAccelerate, boolean manualBrake, boolean manualToggleDoors, boolean manualToggleAto, boolean doorOverride) {
        super(uuid.toString(), ridingCar, x, y, z, isOnGangway, isDriver, manualAccelerate, manualBrake, manualToggleDoors, manualToggleAto, doorOverride);
        this.uuid = uuid;
    }

    public VehicleRidingEntity(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
        this.uuid = UUID.fromString(this.clientId);
    }

    public long getRidingCar() {
        return this.ridingCar;
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getZ() {
        return this.z;
    }

    public boolean getIsOnGangway() {
        return this.isOnGangway;
    }

    public boolean isOnVehicle() {
        return this.ridingCar >= 0L;
    }

    public boolean isDriver() {
        return this.isDriver;
    }

    public boolean manualAccelerate() {
        return this.manualAccelerate;
    }

    public boolean manualBrake() {
        return this.manualBrake;
    }

    public boolean manualToggleDoors() {
        return this.manualToggleDoors;
    }

    public boolean manualToggleAto() {
        return this.manualToggleAto;
    }

    public boolean getDoorOverride() {
        return this.doorOverride;
    }
}

