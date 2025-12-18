package org.mtr.mod.render;

import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.TwoPositionsBase;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.mapping.holder.*;
import org.mtr.mapping.mapper.BlockEntityRenderer;
import org.mtr.mapping.mapper.GraphicsHolder;
import org.mtr.mod.Init;
import org.mtr.mod.block.BlockNode;
import org.mtr.mod.block.BlockSignalBase;
import org.mtr.mod.block.IBlock;
import org.mtr.mod.client.IDrawing;
import org.mtr.mod.client.MinecraftClientData;
import org.mtr.mod.data.IGui;
import org.mtr.mod.data.RailType;

import javax.annotation.Nullable;
import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public abstract class RenderSignalBase<T extends BlockSignalBase.BlockEntityBase> extends BlockEntityRenderer<T> implements IBlock, IGui {

	protected final int aspects;
	private final float colorIndicatorHeight;

	public RenderSignalBase(Argument dispatcher, int colorIndicatorHeight, int aspects) {
		super(dispatcher);
		this.aspects = aspects;
		this.colorIndicatorHeight = colorIndicatorHeight / 16F + SMALL_OFFSET;
	}

	@Override
	public final void render(T entity, float tickDelta, GraphicsHolder graphicsHolder, int light, int overlay) {
		final World world = entity.getWorld2();
		if (world == null) {
			return;
		}

		final ClientPlayerEntity clientPlayerEntity = MinecraftClient.getInstance().getPlayerMapped();
		if (clientPlayerEntity == null) {
			return;
		}

		final BlockPos pos = entity.getPos2();
		final BlockState state = world.getBlockState(pos);
		if (!(state.getBlock().data instanceof BlockSignalBase)) {
			return;
		}

		final float angle = BlockSignalBase.getAngle(state);
		final StoredMatrixTransformations storedMatrixTransformations = new StoredMatrixTransformations(0.5 + entity.getPos2().getX(), entity.getPos2().getY(), 0.5 + entity.getPos2().getZ());
		int redstoneLevel = 0;
		final ObjectArrayList<String> railIds1 = new ObjectArrayList<>();
		final ObjectArrayList<String> railIds2 = new ObjectArrayList<>();

		for (int i = 0; i < (entity.isDoubleSided ? 2 : 1); i++) {
			final float newAngle = angle + i * 180;
			final AspectState aspectState = getAspectState(pos, newAngle + 90);

			if (aspectState != null) {
				final boolean isBackSide = i == 1;
				final IntAVLTreeSet filterColors = entity.getSignalColors(isBackSide);
				final StoredMatrixTransformations storedMatrixTransformationsNew = storedMatrixTransformations.copy();
				storedMatrixTransformationsNew.add(graphicsHolderNew -> graphicsHolderNew.rotateYDegrees(-newAngle));

				if (RenderRails.isHoldingRailRelated(clientPlayerEntity)) {
					final float xStart = -0.015625F * aspectState.detectedColors.size();
					for (int j = 0; j < aspectState.detectedColors.size(); j++) {
						final int signalColor = aspectState.detectedColors.getInt(j);
						final boolean occupied = aspectState.occupiedColors.contains(signalColor);
						final float x = xStart + j * 0.03125F;
						final float width = 0.03125F / (filterColors.isEmpty() || filterColors.contains(signalColor) ? 1 : 8);
						MainRenderer.scheduleRender(new Identifier(Init.MOD_ID, "textures/block/white.png"), false, occupied ? QueuedRenderLayer.EXTERIOR : QueuedRenderLayer.LIGHT, (graphicsHolderNew, offset) -> {
							storedMatrixTransformationsNew.transform(graphicsHolderNew, offset);
							IDrawing.drawTexture(
									graphicsHolderNew,
									x, colorIndicatorHeight, -0.15625F,
									x + 0.03125F, colorIndicatorHeight, -0.15625F,
									x + 0.03125F, colorIndicatorHeight, -0.15625F - width,
									x, colorIndicatorHeight, -0.15625F - width,
									0, 0, 1, 1,
									Direction.UP, occupied ? MainRenderer.getFlashingColor(signalColor, 1) : signalColor | ARGB_BLACK, GraphicsHolder.getDefaultLight()
							);
							graphicsHolderNew.pop();
						});
					}
				}

				// CUSTOM LOGIC START
				// Determine aspect based on lookahead
				int occupiedAspect;

				// 1. Check current block occupancy (Red)
				boolean isCurrentBlockOccupied = filterColors.isEmpty() && aspectState.nodeBlocked || aspectState.occupiedColors.intStream().anyMatch(color -> filterColors.isEmpty() || filterColors.contains(color));

				if (isCurrentBlockOccupied) {
					occupiedAspect = 1; // Red
				} else {
					// Look ahead for next blocks
					occupiedAspect = calculateLookAheadAspect(pos, newAngle + 90, filterColors);
				}
				// CUSTOM LOGIC END

				render(storedMatrixTransformationsNew, entity, tickDelta, occupiedAspect, isBackSide);

				if (occupiedAspect > 0 && occupiedAspect < aspects) {
					redstoneLevel = Math.max(redstoneLevel, (4 - occupiedAspect) * 5);
				}

				(isBackSide ? railIds2 : railIds1).addAll(aspectState.railIds);
			}
		}

		entity.checkForRedstoneUpdate(redstoneLevel, railIds1, railIds2);
	}

	protected abstract void render(StoredMatrixTransformations storedMatrixTransformations, T entity, float tickDelta, int occupiedAspect, boolean isBackSide);

	// New helper method to calculate aspect based on future blocks
	private int calculateLookAheadAspect(BlockPos startBlockPos, float angle, IntAVLTreeSet currentSignalColors) {
		final ClientWorld clientWorld = MinecraftClient.getInstance().getWorldMapped();
		if (clientWorld == null) return 1; // Default to Red if world null

		final BlockPos startNodePos = getNodePos(clientWorld, startBlockPos, Direction.fromRotation(angle));
		if (startNodePos == null) return 1;

		// Start recursive search
		// Returns 3 for Double Yellow, 2 for Yellow, 0 for Green
		return traverseRails(Init.blockPosToPosition(startNodePos), angle, currentSignalColors, 0, 0, new HashSet<>());
	}

	/**
	 * @param currentPos Current node position
	 * @param angle Current angle of travel
	 * @param trackingSignalColors The signal colors of the "current" block we are traversing
	 * @param distanceInCurrentBlock Distance traveled in the current block
	 * @param depth How many blocks ahead we are (0 = current, 1 = next, 2 = next next)
	 * @param visited To prevent infinite loops
	 */
	private int traverseRails(Position currentPos, float angle, IntAVLTreeSet trackingSignalColors, float distanceInCurrentBlock, int depth, Set<Position> visited) {
		if (visited.contains(currentPos)) return 0; // Loop detected, assume Green
		visited.add(currentPos);

		if (depth > 2) return 0; // Lookahead limit reached, assume Green

		final MinecraftClientData clientData = MinecraftClientData.getInstance();
		final Object2ObjectOpenHashMap<Position, org.mtr.core.data.Rail> rails = clientData.positionsToRail.get(currentPos);

		if (rails == null || rails.isEmpty()) return 0; // End of line

		int mostRestrictiveAspect = 0; // Default Green

		for (Map.Entry<Position, Rail> entry : rails.entrySet()) {
			Position endPos = entry.getKey();
			org.mtr.core.data.Rail rail = entry.getValue();

			// Check angle to ensure we don't go backwards
			double railAngle = Math.toDegrees(Math.atan2(endPos.getZ() - currentPos.getZ(), endPos.getX() - currentPos.getX()));
			if (Math.abs(Utilities.circularDifference((long) railAngle, (long) angle, 360)) > 90) continue;

			// Determine if this rail starts a new block
			IntAVLTreeSet railColors = new IntAVLTreeSet();
			rail.getSignalColors().forEach(railColors::add);

			// Check if signal colors match the block we are currently tracking
			boolean isSameBlock;
			if (trackingSignalColors.isEmpty()) {
				isSameBlock = railColors.isEmpty();
			} else {
				// If rail has ANY of the tracking colors, it's considered same block (simplified logic)
				// Or strictly: sets must intersect? MTR usually treats shared colors as same block.
				isSameBlock = false;
				for (int color : trackingSignalColors) {
					if (railColors.contains(color)) {
						isSameBlock = true;
						break;
					}
				}
			}

			// Distance calculation
			float railLength = (float) Math.sqrt(Math.pow(endPos.getX() - currentPos.getX(), 2) + Math.pow(endPos.getZ() - currentPos.getZ(), 2)); // Approximate length

			int branchAspect;
			if (isSameBlock) {
				// Continue in same block
				branchAspect = traverseRails(endPos, (float) railAngle, trackingSignalColors, distanceInCurrentBlock + railLength, depth, new HashSet<>(visited));
			} else {
				// New Block Detected
				// Check Occupancy of this new block
				boolean isOccupied = false;
				for(int color : railColors) {
					if (clientData.railIdToCurrentlyBlockedSignalColors.values().stream().anyMatch(list -> list.contains(color))) {
						isOccupied = true;
						break;
					}
				}

				if (isOccupied) {
					// We found an occupied block ahead
					if (depth == 0) {
						// Block 2 (Next Block) is occupied
						// Check distance of Block 1 (Current Block)
						if (distanceInCurrentBlock < 250) {
							branchAspect = 3; // Double Yellow
						} else {
							branchAspect = 2; // Yellow
						}
					} else {
						// Block 3 (Next Next Block) is occupied
						branchAspect = 2; // Yellow
					}
				} else {
					// Block is clear, continue checking next block
					branchAspect = traverseRails(endPos, (float) railAngle, railColors, 0, depth + 1, new HashSet<>(visited));
				}
			}

			// Take the most restrictive aspect found in any branch
			// Aspects: 1=Red (Handled before), 3=DoubleYellow, 2=Yellow, 0=Green
			// Priority: 3 > 2 > 0 (Red is handled externally)
			if (branchAspect == 3) {
				mostRestrictiveAspect = 3;
			} else if (branchAspect == 2 && mostRestrictiveAspect != 3) {
				mostRestrictiveAspect = 2;
			}
		}

		return mostRestrictiveAspect;
	}


	@Nullable
	public static AspectState getAspectState(BlockPos blockPos, float angle) {
		final ClientWorld clientWorld = MinecraftClient.getInstance().getWorldMapped();
		if (clientWorld == null) {
			return null;
		}

		final BlockPos startPos = getNodePos(clientWorld, blockPos, Direction.fromRotation(angle));
		if (startPos == null) {
			return null;
		}

		final MinecraftClientData minecraftClientData = MinecraftClientData.getInstance();
		final IntArrayList detectedColors = new IntArrayList();
		final IntAVLTreeSet occupiedColors = new IntAVLTreeSet();
		final boolean[] blocked = {false};
		final ObjectArrayList<String> railIds = new ObjectArrayList<>();
		final Position startPosition = Init.blockPosToPosition(startPos);

		minecraftClientData.positionsToRail.getOrDefault(startPosition, new Object2ObjectOpenHashMap<>()).forEach((endPosition, rail) -> {
			if (Math.abs(Utilities.circularDifference(Math.round(Math.toDegrees(Math.atan2(endPosition.getZ() - startPos.getZ(), endPosition.getX() - startPos.getX()))), Math.round(angle), 360)) < 90) {
				rail.getSignalColors().forEach(detectedColors::add);
				final String railId = rail.getHexId();
				minecraftClientData.railIdToCurrentlyBlockedSignalColors.getOrDefault(railId, new LongArrayList()).forEach(color -> occupiedColors.add((int) color));
				if (minecraftClientData.blockedRailIds.contains(TwoPositionsBase.getHexIdRaw(startPosition, endPosition))) {
					blocked[0] = true;
				}
				railIds.add(railId);
			}
		});

		Collections.sort(detectedColors);
		return new AspectState(detectedColors, occupiedColors, blocked[0], railIds);
	}

	@Nullable
	private static BlockPos getNodePos(ClientWorld world, BlockPos pos, Direction facing) {
		int closestDistance = Integer.MAX_VALUE;
		BlockPos closestPos = null;
		for (int z = -4; z <= 4; z++) {
			for (int x = -4; x <= 4; x++) {
				for (int y = -5; y <= 5; y++) {
					final BlockPos checkPos = pos.up(y).offset(facing.rotateYClockwise(), x).offset(facing, z);
					final BlockState checkState = world.getBlockState(checkPos);
					final int distance = checkPos.getManhattanDistance(new Vector3i(pos.data));
					if (checkState.getBlock().data instanceof BlockNode && distance < closestDistance) {
						closestDistance = distance;
						closestPos = checkPos;
					}
				}
			}
		}
		return closestPos;
	}

	public static class AspectState {

		public final IntArrayList detectedColors;
		private final IntAVLTreeSet occupiedColors;
		private final boolean nodeBlocked;
		private final ObjectArrayList<String> railIds;

		private AspectState(IntArrayList detectedColors, IntAVLTreeSet occupiedColors, boolean nodeBlocked, ObjectArrayList<String> railIds) {
			this.detectedColors = detectedColors;
			this.occupiedColors = occupiedColors;
			this.nodeBlocked = nodeBlocked;
			this.railIds = railIds;
		}
	}
}