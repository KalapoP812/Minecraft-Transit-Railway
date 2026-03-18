/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet
 */
package org.mtr.core.data;

import java.util.function.BiFunction;
import java.util.function.Consumer;
import org.mtr.core.data.Data;
import org.mtr.core.data.LiftDirection;
import org.mtr.core.data.LiftFloor;
import org.mtr.core.data.LiftInstruction;
import org.mtr.core.data.Position;
import org.mtr.core.data.TransportMode;
import org.mtr.core.generated.data.LiftSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.Angle;
import org.mtr.core.tool.Utilities;
import org.mtr.core.tool.Vector;
import org.mtr.legacy.data.DataFixer;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet;

public class Lift
extends LiftSchema
implements Utilities {
    private boolean needsUpdate;
    private Position minPosition = new Position(0L, 0L, 0L);
    private Position maxPosition = new Position(0L, 0L, 0L);
    private final LongArrayList distances = new LongArrayList();
    private final boolean isClientside;
    private static final float MAX_SPEED = 0.01f;
    private static final int DOOR_OPEN_TIME = 2000;
    private static final int DOOR_MOVE_TIME = 1600;
    private static final int DOOR_DELAY = 500;
    private static final int STOPPING_TIME = 5700;

    public Lift(Data data) {
        super(TransportMode.values()[0], data);
        this.isClientside = !(data instanceof Simulator);
    }

    public Lift(ReaderBase readerBase, Data data) {
        super(DataFixer.convertLift(readerBase), data);
        this.isClientside = !(data instanceof Simulator);
        this.updateData(readerBase);
    }

    @Override
    public void updateData(ReaderBase readerBase) {
        super.updateData(readerBase);
        this.setDistances();
        this.needsUpdate = true;
    }

    @Override
    public boolean isValid() {
        return !this.floors.isEmpty();
    }

    public void tick(long millisElapsed) {
        if (this.stoppingCoolDown > 0L) {
            this.stoppingCoolDown = Math.max(this.stoppingCoolDown - millisElapsed, 0L);
            if (this.stoppingCoolDown == 0L) {
                if (this.isClientside) {
                    this.stoppingCoolDown = 1L;
                } else {
                    this.needsUpdate = true;
                }
            }
        } else {
            if (this.instructions.isEmpty()) {
                this.speed = Math.max(Math.abs(this.speed) - 4.0E-6 * (double)millisElapsed, 0.0) * Math.signum(this.speed);
            } else {
                long nextInstructionProgress = this.getProgress(((LiftInstruction)this.instructions.get(0)).getFloor());
                this.speed = this.speed * this.speed / 2.0 / 4.0E-6 > Math.abs((double)nextInstructionProgress - this.railProgress) ? Math.max(Math.abs(this.speed) - 4.0E-6 * (double)millisElapsed, 4.0E-6) * Math.signum(this.speed) : Utilities.clamp(this.speed + 4.0E-6 * (double)millisElapsed * Math.signum((double)nextInstructionProgress - this.railProgress), (double)-0.01f, (double)0.01f);
                if (Math.abs(this.railProgress - (double)nextInstructionProgress) <= Math.abs(this.speed * (double)millisElapsed)) {
                    this.railProgress = nextInstructionProgress;
                    this.speed = 0.0;
                    if (!this.isClientside) {
                        this.instructions.remove(0);
                        this.stoppingCoolDown = 5700L;
                        this.needsUpdate = true;
                    }
                }
            }
            this.railProgress = Utilities.clamp(this.railProgress + this.speed * (double)millisElapsed, 0.0, (double)this.getProgress(Integer.MAX_VALUE));
        }
        if (this.data instanceof Simulator) {
            ((Simulator)this.data).clients.forEach(client -> {
                if (Utilities.isBetween(client.getPosition(), this.minPosition, this.maxPosition, client.getUpdateRadius())) {
                    client.update(this, this.needsUpdate);
                }
            });
            this.needsUpdate = false;
        }
    }

    public double pressButton(LiftInstruction liftInstruction, boolean add) {
        if (this.isClientside) {
            return -1.0;
        }
        int buttonFloor = liftInstruction.getFloor();
        if (buttonFloor < 0 || buttonFloor >= this.floors.size()) {
            return -1.0;
        }
        long buttonProgress = this.getProgress(buttonFloor);
        LiftDirection buttonDirection = liftInstruction.getDirection();
        LiftDirection tempDirection = this.getDirection();
        double tempProgress = this.railProgress + this.speed * this.speed / 2.0 / 4.0E-6 * Math.signum(this.speed);
        double distance = 0.0;
        for (int i = 0; i < this.instructions.size(); ++i) {
            LiftInstruction nextInstruction = (LiftInstruction)this.instructions.get(i);
            if (liftInstruction.equals(nextInstruction)) {
                return -1.0;
            }
            long nextInstructionProgress = this.getProgress(nextInstruction.getFloor());
            LiftDirection nextInstructionDirection = nextInstruction.getDirection();
            LiftDirection directionToNextFloor = LiftDirection.fromDifference((double)nextInstructionProgress - tempProgress);
            boolean condition1 = (buttonDirection == LiftDirection.NONE || buttonDirection == directionToNextFloor) && Utilities.isBetween(buttonProgress, tempProgress, nextInstructionProgress);
            boolean condition2 = nextInstructionDirection != LiftDirection.NONE && nextInstructionDirection != directionToNextFloor && LiftDirection.fromDifference(buttonProgress - nextInstructionProgress) == (directionToNextFloor == LiftDirection.NONE ? tempDirection : directionToNextFloor);
            if (condition1 || condition2) {
                if (add) {
                    this.instructions.add(i, liftInstruction);
                    this.needsUpdate = true;
                }
                return distance + Math.abs((double)buttonProgress - tempProgress);
            }
            distance += Math.abs((double)nextInstructionProgress - tempProgress);
            tempDirection = directionToNextFloor;
            tempProgress = nextInstructionProgress;
        }
        if (add) {
            this.instructions.add(liftInstruction);
            this.needsUpdate = true;
        }
        return distance + Math.abs((double)buttonProgress - tempProgress);
    }

    public double getHeight() {
        return this.height;
    }

    public double getWidth() {
        return this.width;
    }

    public double getDepth() {
        return this.depth;
    }

    public double getOffsetX() {
        return this.offsetX;
    }

    public double getOffsetY() {
        return this.offsetY;
    }

    public double getOffsetZ() {
        return this.offsetZ;
    }

    public boolean getIsDoubleSided() {
        return this.isDoubleSided;
    }

    public String getStyle() {
        return this.style;
    }

    public Angle getAngle() {
        return this.angle;
    }

    public Vector getPosition(BiFunction<Position, Position, ObjectArrayList<Vector>> trackProvider) {
        return this.currentFloorCallback((percentage, index) -> {
            ObjectArrayList trackPositions = (ObjectArrayList)trackProvider.apply(((LiftFloor)this.floors.get(index - 1)).getPosition(), ((LiftFloor)this.floors.get(index)).getPosition());
            double progress = percentage * (double)(trackPositions.size() - 1);
            int trackIndex = (int)Math.floor(progress);
            double trackPercentage = progress - (double)trackIndex;
            Vector position1 = Utilities.getElement(trackPositions, trackIndex, new Vector(0.0, 0.0, 0.0));
            Vector position2 = Utilities.getElement(trackPositions, trackIndex + 1, new Vector(0.0, 0.0, 0.0));
            return new Vector(Lift.getValueFromPercentage(trackPercentage, position1.x(), position2.x()), Lift.getValueFromPercentage(trackPercentage, position1.y(), position2.y()), Lift.getValueFromPercentage(trackPercentage, position1.z(), position2.z()));
        }, new Vector(0.0, 0.0, 0.0));
    }

    public float getDoorValue() {
        if (this.stoppingCoolDown < 500L) {
            return 0.0f;
        }
        if (this.stoppingCoolDown < 2100L) {
            return (float)(this.stoppingCoolDown - 500L) / 1600.0f;
        }
        if (this.stoppingCoolDown <= 4100L) {
            return 1.0f;
        }
        return (float)(5700L - this.stoppingCoolDown) / 1600.0f;
    }

    public boolean hasCoolDown() {
        return this.stoppingCoolDown > 0L;
    }

    public void iterateFloors(Consumer<LiftFloor> consumer) {
        this.floors.forEach(consumer);
    }

    public int getFloorCount() {
        return this.floors.size();
    }

    public ObjectArraySet<LiftDirection> hasInstruction(int floor) {
        ObjectArraySet liftDirections = new ObjectArraySet();
        this.instructions.forEach(liftInstruction -> {
            if (liftInstruction.getFloor() == floor) {
                liftDirections.add(liftInstruction.getDirection());
            }
        });
        return liftDirections;
    }

    public LiftFloor getCurrentFloor() {
        return this.currentFloorCallback((percentage, index) -> (LiftFloor)this.floors.get(index - (percentage < 0.5 ? 1 : 0)), new LiftFloor(new Position(0L, 0L, 0L)));
    }

    public LiftDirection getDirection() {
        if (this.instructions.isEmpty()) {
            return LiftDirection.NONE;
        }
        return LiftDirection.fromDifference((double)this.getProgress(((LiftInstruction)this.instructions.get(0)).getFloor()) - this.railProgress);
    }

    public void setDimensions(double height, double width, double depth, double offsetX, double offsetY, double offsetZ) {
        this.height = height;
        this.width = width;
        this.depth = depth;
        this.offsetX = offsetX;
        this.offsetY = offsetY;
        this.offsetZ = offsetZ;
    }

    public void setIsDoubleSided(boolean isDoubleSided) {
        this.isDoubleSided = isDoubleSided;
    }

    public void setStyle(String style) {
        this.style = style;
    }

    public void setAngle(Angle angle) {
        this.angle = angle;
    }

    public void setFloors(ObjectArrayList<LiftFloor> liftFloors) {
        this.floors.clear();
        this.floors.addAll(liftFloors);
        this.instructions.clear();
        this.setDistances();
        this.needsUpdate = true;
    }

    public void setFloors(Lift lift) {
        if (lift != this) {
            this.setFloors((ObjectArrayList<LiftFloor>)lift.floors);
        }
    }

    public int getFloorIndex(Position position) {
        for (int i = 0; i < this.floors.size(); ++i) {
            if (!position.equals(((LiftFloor)this.floors.get(i)).getPosition())) continue;
            return i;
        }
        return -1;
    }

    public boolean overlappingFloors(Lift lift) {
        return lift.floors.stream().anyMatch(liftFloor -> this.getFloorIndex(liftFloor.getPosition()) >= 0);
    }

    public void updateFloor(LiftFloor liftFloor) {
        this.floors.forEach(currentFloor -> {
            if (currentFloor.getPosition().equals(liftFloor.getPosition())) {
                currentFloor.setNumberAndDescription(liftFloor.getNumber(), liftFloor.getDescription());
            }
        });
    }

    private void setDistances() {
        this.distances.clear();
        long minX = Long.MAX_VALUE;
        long maxX = -9223372036854775807L;
        long minY = Long.MAX_VALUE;
        long maxY = -9223372036854775807L;
        long minZ = Long.MAX_VALUE;
        long maxZ = -9223372036854775807L;
        long distance = 0L;
        for (int i = 0; i < this.floors.size(); ++i) {
            Position position = ((LiftFloor)this.floors.get(i)).getPosition();
            minX = Math.min(minX, position.getX());
            maxX = Math.max(maxX, position.getX());
            minY = Math.min(minY, position.getY());
            maxY = Math.max(maxY, position.getY());
            minZ = Math.min(minZ, position.getZ());
            maxZ = Math.max(maxZ, position.getZ());
            if (i == 0) {
                this.distances.add(0L);
                continue;
            }
            this.distances.add(distance += position.manhattanDistance(((LiftFloor)this.floors.get(i - 1)).getPosition()));
        }
        this.minPosition = new Position(minX, minY, minZ);
        this.maxPosition = new Position(maxX, maxY, maxZ);
    }

    private long getProgress(int floor) {
        return this.distances.isEmpty() ? 0L : this.distances.getLong(Utilities.clamp(floor, 0, this.distances.size() - 1));
    }

    private <T> T currentFloorCallback(PercentageCallback<T> percentageCallback, T defaultValue) {
        for (int i = 1; i < Math.min(this.distances.size(), this.floors.size()); ++i) {
            long distance2;
            long distance1 = this.distances.getLong(i - 1);
            if (!Utilities.isBetween(this.railProgress, distance1, distance2 = this.distances.getLong(i))) continue;
            return percentageCallback.apply((this.railProgress - (double)distance1) / (double)(distance2 - distance1), i);
        }
        return defaultValue;
    }

    private static double getValueFromPercentage(double percentage, double value1, double value2) {
        return percentage * (value2 - value1) + value1;
    }

    @FunctionalInterface
    private static interface PercentageCallback<T> {
        public T apply(double var1, int var3);
    }
}

