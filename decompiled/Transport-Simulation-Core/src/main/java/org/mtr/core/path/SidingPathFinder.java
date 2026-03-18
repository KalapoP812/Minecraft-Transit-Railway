/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectOpenHashSet
 */
package org.mtr.core.path;

import java.util.function.BiConsumer;
import javax.annotation.Nullable;
import org.mtr.core.data.AreaBase;
import org.mtr.core.data.Data;
import org.mtr.core.data.NameColorDataBase;
import org.mtr.core.data.PathData;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.SavedRailBase;
import org.mtr.core.data.TransportMode;
import org.mtr.core.path.PathFinder;
import org.mtr.core.tool.Angle;
import org.mtr.core.tool.Utilities;
import org.mtr.core.tool.Vector;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectOpenHashSet;

public final class SidingPathFinder<T extends AreaBase<T, U>, U extends SavedRailBase<U, T>, V extends AreaBase<V, W>, W extends SavedRailBase<W, V>>
extends PathFinder<SidingPathFinder.PositionAndAngle> {
    public final U startSavedRail;
    public final W endSavedRail;
    public final int stopIndex;
    private final TransportMode transportMode;
    private final Object2ObjectOpenHashMap<Position, Object2ObjectOpenHashMap<Position, Rail>> positionsToRail;
    private final Object2ObjectOpenHashMap<Position, Rail> runwaysInbound;
    private final ObjectOpenHashSet<Position> runwaysOutbound;
    public static final int AIRPLANE_SPEED = 300;
    private static final int MAX_AIRPLANE_TURN_ARC = 128;

    public SidingPathFinder(Data data, U startSavedRail, W endSavedRail, int stopIndex) {
        super(new PositionAndAngle(((SavedRailBase)startSavedRail).getRandomPosition(), null), new PositionAndAngle(((SavedRailBase)endSavedRail).getRandomPosition(), null));
        this.transportMode = ((NameColorDataBase)startSavedRail).getTransportMode();
        this.positionsToRail = data.positionsToRail;
        this.runwaysInbound = data.runwaysInbound;
        this.runwaysOutbound = data.runwaysOutbound;
        this.startSavedRail = startSavedRail;
        this.endSavedRail = endSavedRail;
        this.stopIndex = stopIndex;
    }

    @Override
    protected ObjectArrayList<PathFinder.ConnectionDetails<PositionAndAngle>> getConnections(long elapsedTime, PositionAndAngle node, @Nullable Long previousRouteId) {
        ObjectArrayList<PathFinder.ConnectionDetails<PositionAndAngle>> connections = new ObjectArrayList<>();
        Object2ObjectOpenHashMap<Position, Rail> railConnections = this.positionsToRail.get(node.position);
        if (railConnections != null) {
            railConnections.forEach((position, rail) -> {
                double speedLimit = rail.getSpeedLimitMetersPerMillisecond(node.position);
                if (speedLimit > 0.0 && (node.angle == null || node.angle == rail.getStartAngle(node.position) || rail.canTurnBack())) {
                    connections.add(new PathFinder.ConnectionDetails<PositionAndAngle>(new PositionAndAngle((Position)position, rail.getStartAngle((Position)position).getOpposite()), Math.round(rail.railMath.getLength() / speedLimit), 0L, 0L));
                }
            });
        }
        if (this.transportMode == TransportMode.AIRPLANE && this.runwaysOutbound.contains(node.position)) {
            this.runwaysInbound.forEach((position, rail) -> connections.add(new PathFinder.ConnectionDetails<PositionAndAngle>(new PositionAndAngle((Position)position, rail.getStartAngle((Position)position)), 1L, 0L, 0L)));
        }
        return connections;
    }

    @Override
    protected long getWeightFromEndNode(PositionAndAngle node) {
        return node.position.manhattanDistance(((PositionAndAngle)this.endNode).position);
    }

    @Nullable
    private ObjectArrayList<PathData> tick(long cruisingAltitude) {
        ObjectArrayList<PathFinder.ConnectionDetails<PositionAndAngle>> connectionDetailsList = this.findPath();
        if (connectionDetailsList == null) {
            return null;
        }
        if (connectionDetailsList.isEmpty()) {
            return new ObjectArrayList();
        }
        this.padConnectionDetailsList((ObjectArrayList<PathFinder.ConnectionDetails<PositionAndAngle>>)connectionDetailsList, (SavedRailBase)this.startSavedRail, false);
        this.padConnectionDetailsList((ObjectArrayList<PathFinder.ConnectionDetails<PositionAndAngle>>)connectionDetailsList, (SavedRailBase)this.endSavedRail, true);
        ObjectArrayList<PathData> path = new ObjectArrayList<>();
        for (int i = 1; i < connectionDetailsList.size(); ++i) {
            Position position2;
            Position position1 = ((PositionAndAngle)((PathFinder.ConnectionDetails)connectionDetailsList.get((int)(i - 1))).node).position;
            Rail rail = (Rail)Data.tryGet(this.positionsToRail, position1, position2 = ((PositionAndAngle)((PathFinder.ConnectionDetails)connectionDetailsList.get((int)i)).node).position);
            if (rail == null) {
                Angle angle1 = ((PositionAndAngle)((PathFinder.ConnectionDetails)connectionDetailsList.get((int)(i - 1))).node).angle;
                Angle angle2 = ((PositionAndAngle)((PathFinder.ConnectionDetails)connectionDetailsList.get((int)i)).node).angle;
                if (this.transportMode == TransportMode.AIRPLANE && angle1 != null && angle2 != null) {
                    long heightDifference1 = cruisingAltitude - position1.getY();
                    long heightDifference2 = cruisingAltitude - position2.getY();
                    Position cruisingPosition1 = position1.offset(Math.round(angle1.cos * (double)Math.abs(heightDifference1) * 4.0), heightDifference1, Math.round(angle1.sin * (double)Math.abs(heightDifference1) * 4.0));
                    Position cruisingPosition4 = position2.offset(Math.round(-angle2.cos * (double)Math.abs(heightDifference2) * 4.0), heightDifference2, Math.round(-angle2.sin * (double)Math.abs(heightDifference2) * 4.0));
                    long turnArc = Math.min(128L, cruisingPosition1.manhattanDistance(cruisingPosition4) / 8L);
                    path.add(SidingPathFinder.getAirplanePathData(position1, angle1, cruisingPosition1, angle1.getOpposite(), this.stopIndex));
                    Angle expectedAngle = Angle.fromAngle((float)Math.toDegrees(Math.atan2(cruisingPosition4.getZ() - cruisingPosition1.getZ(), cruisingPosition4.getX() - cruisingPosition1.getX())));
                    Position cruisingPosition2 = SidingPathFinder.addAirplanePath(angle1, cruisingPosition1, expectedAngle, turnArc, (ObjectArrayList<PathData>)path, this.stopIndex, false);
                    ObjectArrayList tempRailData = new ObjectArrayList();
                    Position cruisingPosition3 = SidingPathFinder.addAirplanePath(angle2.getOpposite(), cruisingPosition4, expectedAngle.getOpposite(), turnArc, (ObjectArrayList<PathData>)tempRailData, this.stopIndex, true);
                    path.add(SidingPathFinder.getAirplanePathData(cruisingPosition2, expectedAngle, cruisingPosition3, expectedAngle.getOpposite(), this.stopIndex));
                    path.addAll((ObjectList)tempRailData);
                    path.add(SidingPathFinder.getAirplanePathData(cruisingPosition4, angle2, position2, angle2.getOpposite(), this.stopIndex));
                    continue;
                }
                return new ObjectArrayList();
            }
            if (i == connectionDetailsList.size() - 1) {
                path.add(new PathData(rail, ((NameColorDataBase)this.endSavedRail).getId(), this.endSavedRail instanceof Platform ? ((Platform)this.endSavedRail).getDwellTime() : 1L, this.stopIndex + 1, position1, position2));
                continue;
            }
            if (rail.canTurnBack() && ((PositionAndAngle)((PathFinder.ConnectionDetails)connectionDetailsList.get((int)(i + 1))).node).position.equals(position1)) {
                path.add(new PathData(rail, 0L, 1L, this.stopIndex, position1, position2));
                continue;
            }
            path.add(new PathData(rail, 0L, 0L, this.stopIndex, position1, position2));
        }
        return path;
    }

    public static <T extends AreaBase<T, U>, U extends SavedRailBase<U, T>, V extends AreaBase<V, W>, W extends SavedRailBase<W, V>> void findPathTick(ObjectArrayList<PathData> path, ObjectArrayList<SidingPathFinder<T, U, V, W>> sidingPathFinders, long cruisingAltitude, Runnable callbackSuccess, BiConsumer<U, W> callbackFail) {
        if (!sidingPathFinders.isEmpty()) {
            Utilities.loopUntilTimeout(() -> {
                SidingPathFinder<T, U, V, W> sidingPathFinder = sidingPathFinders.get(0);
                ObjectArrayList<PathData> tempPath = sidingPathFinder.tick(cruisingAltitude);
                if (tempPath != null) {
                    if (tempPath.size() < 2) {
                        sidingPathFinders.clear();
                        path.clear();
                        callbackFail.accept(sidingPathFinder.startSavedRail, sidingPathFinder.endSavedRail);
                        return 0;
                    }
                    if (SidingPathFinder.overlappingPaths(path, tempPath)) {
                        tempPath.remove(0);
                    }
                    path.addAll(tempPath);
                    sidingPathFinders.remove(0);
                    if (sidingPathFinders.isEmpty()) {
                        callbackSuccess.run();
                        return 0;
                    }
                }
                return null;
            }, 5L);
        }
    }

    public static void generatePathDataDistances(ObjectArrayList<PathData> path, double initialDistance) {
        ObjectArrayList<PathData> tempPath = new ObjectArrayList<>(path);
        double endDistance = initialDistance;
        path.clear();
        for (PathData oldPathData : tempPath) {
            double startDistance = endDistance;
            path.add(new PathData(oldPathData, startDistance, endDistance += oldPathData.getRailLength()));
        }
    }

    public static boolean overlappingPaths(ObjectArrayList<PathData> path, ObjectArrayList<PathData> newPath) {
        if (path.isEmpty() || newPath.isEmpty()) {
            return false;
        }
        return ((PathData)Utilities.getElement(path, -1)).isSameRail((PathData)newPath.get(0));
    }

    private <X extends AreaBase<X, Y>, Y extends SavedRailBase<Y, X>> void padConnectionDetailsList(ObjectArrayList<PathFinder.ConnectionDetails<PositionAndAngle>> connectionDetailsList, SavedRailBase<Y, X> savedRail, boolean isEnd) {
        Position lastPosition = ((PositionAndAngle)((PathFinder.ConnectionDetails)Utilities.getElement(connectionDetailsList, (int)(isEnd ? -1 : 0))).node).position;
        if (!savedRail.containsPos(lastPosition)) {
            this.positionsToRail.get(lastPosition).keySet().stream().filter(savedRail::containsPos).findFirst().ifPresent(newPosition -> {
                connectionDetailsList.add(isEnd ? connectionDetailsList.size() : 0, new PathFinder.ConnectionDetails<PositionAndAngle>(new PositionAndAngle((Position)newPosition, null), 0L, 0L, 0L));
                connectionDetailsList.add(isEnd ? connectionDetailsList.size() : 0, new PathFinder.ConnectionDetails<PositionAndAngle>(new PositionAndAngle(savedRail.getOtherPosition((Position)newPosition), null), 0L, 0L, 0L));
            });
        } else if (connectionDetailsList.size() > 1 && !savedRail.containsPos(((PositionAndAngle)((PathFinder.ConnectionDetails)Utilities.getElement(connectionDetailsList, (int)(isEnd ? -2 : 1))).node).position)) {
            connectionDetailsList.add(isEnd ? connectionDetailsList.size() : 0, new PathFinder.ConnectionDetails<PositionAndAngle>(new PositionAndAngle(savedRail.getOtherPosition(lastPosition), null), 0L, 0L, 0L));
        }
    }

    private static Position addAirplanePath(Angle startAngle, Position startPos, Angle expectedAngle, long turnArc, ObjectArrayList<PathData> tempRailPath, int stopIndex, boolean reverse) {
        Angle angleDifference = expectedAngle.sub(startAngle);
        boolean turnRight = angleDifference.angleRadians > 0.0;
        Angle tempAngle = startAngle;
        Position tempPos = startPos;
        for (int i = 0; i < Angle.values().length && tempAngle != expectedAngle; ++i) {
            Angle oldTempAngle = tempAngle;
            Position oldTempPos = tempPos;
            Angle rotateAngle = turnRight ? Angle.SEE : Angle.NEE;
            tempAngle = tempAngle.add(rotateAngle);
            Vector posOffset = new Vector(turnArc, 0.0, 0.0).rotateY(-oldTempAngle.angleRadians - rotateAngle.angleRadians / 2.0);
            tempPos = oldTempPos.offset(Math.round(posOffset.x()), Math.round(posOffset.y()), Math.round(posOffset.z()));
            if (reverse) {
                tempRailPath.add(0, SidingPathFinder.getAirplanePathData(tempPos, tempAngle.getOpposite(), oldTempPos, oldTempAngle, stopIndex));
                continue;
            }
            tempRailPath.add(SidingPathFinder.getAirplanePathData(oldTempPos, oldTempAngle, tempPos, tempAngle.getOpposite(), stopIndex));
        }
        return tempPos;
    }

    private static PathData getAirplanePathData(Position position1, Angle angle1, Position position2, Angle angle2, int stopIndex) {
        return new PathData(Rail.newRail(position1, angle1, position2, angle2, Rail.Shape.QUADRATIC, 0.0, (ObjectArrayList<String>)new ObjectArrayList(), 300L, 0L, false, false, true, false, false, TransportMode.AIRPLANE), 0L, 0L, stopIndex, position1, position2);
    }

    protected static class PositionAndAngle {
        private final Position position;
        @Nullable
        private final Angle angle;

        private PositionAndAngle(Position position, @Nullable Angle angle) {
            this.position = position;
            this.angle = angle;
        }

        public boolean equals(Object obj) {
            if (obj instanceof PositionAndAngle) {
                return this.position.equals(((PositionAndAngle)obj).position) && (this.angle == null || ((PositionAndAngle)obj).angle == null || this.angle == ((PositionAndAngle)obj).angle);
            }
            return super.equals(obj);
        }

        public int hashCode() {
            return this.position.hashCode() ^ (this.angle == null ? 0 : this.angle.ordinal() + 1) << 8;
        }
    }
}

