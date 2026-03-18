/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2LongOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.path;

import javax.annotation.Nullable;
import org.mtr.core.Main;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2LongOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public abstract class PathFinder<T> {
    private long totalTime = Long.MAX_VALUE;
    private boolean completed;
    private int iterations;
    protected final T startNode;
    protected final T endNode;
    private final Object2LongOpenHashMap<T> globalBlacklist = new Object2LongOpenHashMap();
    private final Object2LongOpenHashMap<T> localBlacklist = new Object2LongOpenHashMap();
    private final ObjectArrayList<ConnectionDetails<T>> tempData;
    private final ObjectArrayList<ConnectionDetails<T>> data = new ObjectArrayList();

    public PathFinder(T startNode, T endNode) {
        this.startNode = startNode;
        this.endNode = endNode;
        this.completed = startNode.equals(endNode);
        this.tempData = new ObjectArrayList<>();
        this.tempData.add(new ConnectionDetails<>(startNode, 0L, 0L, 0L));
    }

    protected ObjectArrayList<ConnectionDetails<T>> findPath() {
        if (!this.completed) {
            long elapsedTime = this.tempData.stream().mapToLong(data -> data.duration + data.waitingTime).sum();
            ConnectionDetails<T> prevConnectionDetails = (ConnectionDetails<T>)Utilities.getElement(this.tempData, -1);
            T prevNode = prevConnectionDetails == null ? this.startNode : prevConnectionDetails.node;
            T bestNode = null;
            long bestIncrease = Long.MIN_VALUE;
            long bestDuration = 0L;
            long bestWaitingTime = 0L;
            long bestRouteId = 0L;
            for (ConnectionDetails<T> connectionDetails : this.getConnections(elapsedTime, prevNode, prevConnectionDetails == null ? null : Long.valueOf(prevConnectionDetails.routeId))) {
                T thisNode = connectionDetails.node;
                long duration = connectionDetails.duration;
                long waitingTime = connectionDetails.waitingTime;
                long totalDuration = duration + waitingTime;
                if (!this.verifyTime(thisNode, elapsedTime + totalDuration)) continue;
                long increase = (this.getWeightFromEndNode(prevNode) - this.getWeightFromEndNode(thisNode)) / totalDuration;
                this.globalBlacklist.put(thisNode, elapsedTime + totalDuration);
                if (increase <= bestIncrease) continue;
                bestNode = thisNode;
                bestIncrease = increase;
                bestDuration = duration;
                bestWaitingTime = waitingTime;
                bestRouteId = connectionDetails.routeId;
            }
            if (bestNode == null || bestDuration == 0L) {
                if (this.tempData.isEmpty()) {
                    this.completed = true;
                    Main.LOGGER.debug("Found the best path after {} iteration(s)", this.iterations);
                } else {
                    this.tempData.remove(this.tempData.size() - 1);
                }
            } else {
                long totalDuration = elapsedTime + bestDuration + bestWaitingTime;
                this.localBlacklist.put(bestNode, totalDuration);
                this.tempData.add(new ConnectionDetails<>(bestNode, bestDuration, bestWaitingTime, bestRouteId));
                if (bestNode.equals(this.endNode)) {
                    if (totalDuration > 0L && (totalDuration < this.totalTime || totalDuration == this.totalTime && this.tempData.size() < this.data.size())) {
                        if (totalDuration == this.totalTime) {
                            Main.LOGGER.debug("Found a shorter path!");
                        }
                        this.totalTime = totalDuration;
                        this.data.clear();
                        this.data.addAll(this.tempData);
                        ++this.iterations;
                    }
                    this.tempData.clear();
                    this.localBlacklist.clear();
                }
            }
        }
        return this.completed ? this.data : null;
    }

    protected abstract ObjectArrayList<ConnectionDetails<T>> getConnections(long var1, T var3, @Nullable Long var4);

    protected abstract long getWeightFromEndNode(T var1);

    private boolean verifyTime(T node, long time) {
        return time < this.totalTime && PathFinder.compareBlacklist(this.localBlacklist, node, time, false) && PathFinder.compareBlacklist(this.globalBlacklist, node, time, true);
    }

    private static <U> boolean compareBlacklist(Object2LongOpenHashMap<U> blacklist, U node, long time, boolean lessThanOrEqualTo) {
        return lessThanOrEqualTo ? time <= blacklist.getOrDefault(node, Long.MAX_VALUE) : time < blacklist.getOrDefault(node, Long.MAX_VALUE);
    }

    protected static class ConnectionDetails<T> {
        public final T node;
        public final long duration;
        public final long waitingTime;
        public final long routeId;

        protected ConnectionDetails(T node, long duration, long waitingTime, long routeId) {
            this.node = node;
            this.duration = Math.max(1L, duration);
            this.waitingTime = waitingTime;
            this.routeId = routeId;
        }
    }
}

