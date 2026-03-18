/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet
 */
package org.mtr.legacy.data;

import java.nio.file.Path;
import java.util.UUID;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.SignalModification;
import org.mtr.core.data.TransportMode;
import org.mtr.core.simulation.FileLoader;
import org.mtr.core.tool.Angle;
import org.mtr.legacy.data.DataFixer;
import org.mtr.legacy.data.LegacyRailNode;
import org.mtr.legacy.data.LegacySignalBlock;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet;

public final class LegacyRailLoader {
    public static void load(Path savePath, ObjectArraySet<Rail> rails, boolean threadedFileLoading) {
        ObjectArraySet<LegacyRailNode> legacyRailNodes = new ObjectArraySet<>();
        ObjectArraySet<LegacySignalBlock> legacySignalBlocks = new ObjectArraySet<>();
        new FileLoader<LegacyRailNode>((ObjectSet<LegacyRailNode>)legacyRailNodes, LegacyRailNode::new, savePath, "rails", threadedFileLoading);
        new FileLoader<LegacySignalBlock>((ObjectSet<LegacySignalBlock>)legacySignalBlocks, LegacySignalBlock::new, savePath, "signal-blocks", threadedFileLoading);
        Object2ObjectOpenHashMap<UUID, DataFixer.RailType> railCache = new Object2ObjectOpenHashMap<>();
        legacyRailNodes.forEach(legacyRailNode -> {
            Position startPosition = legacyRailNode.getStartPosition();
            long startPositionLong = legacyRailNode.getStartPositionLong();
            legacyRailNode.iterateConnections(railNodeConnection -> {
                DataFixer.RailType railType = railNodeConnection.getRailType();
                Position endPosition = railNodeConnection.getEndPosition();
                long endPositionLong = railNodeConnection.getEndPositionLong();
                Angle startAngle = railNodeConnection.getStartAngle();
                Angle endAngle = railNodeConnection.getEndAngle();
                TransportMode transportMode = railNodeConnection.getTransportMode();
                String modelKey = railNodeConnection.getModelKey();
                ObjectArrayList styles = modelKey.isEmpty() ? (transportMode == TransportMode.BOAT ? new ObjectArrayList() : ObjectArrayList.of((Object[])new String[]{"default"})) : (modelKey.equals("null") ? new ObjectArrayList() : ObjectArrayList.of((Object[])new String[]{String.format("%s_%s", modelKey, railNodeConnection.getIsSecondaryDirection() ? 2 : 1)}));
                double verticalRadius = railNodeConnection.getVerticalRadius();
                UUID uuid = LegacyRailLoader.getUuid(startPositionLong, endPositionLong);
                DataFixer.RailType oldRailType = railCache.get(uuid);
                if (oldRailType != null) {
                    Rail rail;
                    switch (railType) {
                        case PLATFORM: {
                            rail = Rail.newPlatformRail(startPosition, startAngle, endPosition, endAngle, verticalRadius == 0.0 ? Rail.Shape.QUADRATIC : Rail.Shape.TWO_RADII, Math.max(verticalRadius, 0.0), (ObjectArrayList<String>)styles, transportMode);
                            break;
                        }
                        case SIDING: {
                            rail = Rail.newSidingRail(startPosition, startAngle, endPosition, endAngle, verticalRadius == 0.0 ? Rail.Shape.QUADRATIC : Rail.Shape.TWO_RADII, Math.max(verticalRadius, 0.0), (ObjectArrayList<String>)styles, transportMode);
                            break;
                        }
                        case TURN_BACK: {
                            rail = Rail.newTurnBackRail(startPosition, startAngle, endPosition, endAngle, verticalRadius == 0.0 ? Rail.Shape.QUADRATIC : Rail.Shape.TWO_RADII, Math.max(verticalRadius, 0.0), (ObjectArrayList<String>)styles, transportMode);
                            break;
                        }
                        default: {
                            Rail.Shape shape = railType == DataFixer.RailType.CABLE_CAR || oldRailType == DataFixer.RailType.CABLE_CAR ? Rail.Shape.CABLE : (verticalRadius == 0.0 ? Rail.Shape.QUADRATIC : Rail.Shape.TWO_RADII);
                            rail = Rail.newRail(startPosition, startAngle, endPosition, endAngle, shape, Math.max(verticalRadius, 0.0), (ObjectArrayList<String>)styles, railType.speedLimitKilometersPerHour, oldRailType.speedLimitKilometersPerHour, false, false, true, railType == DataFixer.RailType.RUNWAY, true, transportMode);
                        }
                    }
                    SignalModification signalModification = new SignalModification(startPosition, endPosition, false);
                    legacySignalBlocks.forEach(legacySignalBlock -> {
                        if (legacySignalBlock.isRail(startPosition, endPosition)) {
                            signalModification.putColorToAdd(legacySignalBlock.getColor());
                        }
                    });
                    rail.applyModification(signalModification);
                    rails.add(rail);
                } else {
                    railCache.put(uuid, railType);
                }
            });
        });
    }

    private static UUID getUuid(long value1, long value2) {
        return value1 > value2 ? new UUID(value1, value2) : new UUID(value2, value1);
    }
}

