package org.mtr.mod.block;

import org.mtr.core.data.Rail;
import org.mtr.core.data.TransportMode;
import org.mtr.core.tool.Angle;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair;
import org.mtr.mapping.holder.*;
import org.mtr.mapping.mapper.BlockEntityExtension;
import org.mtr.mapping.mapper.BlockWithEntity;
import org.mtr.mapping.mapper.DirectionHelper;
import org.mtr.mapping.mapper.TextHelper;
import org.mtr.mapping.tool.HolderBase;
import org.mtr.mod.BlockEntityTypes;
import org.mtr.mod.Init;
import org.mtr.mod.Items;
import org.mtr.mod.client.MinecraftClientData;
import org.mtr.mod.generated.lang.TranslationProvider;
import org.mtr.mod.packet.ClientPacketHelper;
import org.mtr.mod.packet.PacketDeleteData;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.List;

public class BlockNode extends BlockWaterloggable implements DirectionHelper, Waterloggable, BlockWithEntity {

	public final TransportMode transportMode;

	public static final BooleanProperty FACING = BooleanProperty.of("facing");
	public static final BooleanProperty IS_22_5 = BooleanProperty.of("is_22_5");
	public static final BooleanProperty IS_45 = BooleanProperty.of("is_45");
	public static final BooleanProperty IS_CONNECTED = BooleanProperty.of("is_connected");

	// Allows for ghost rails to use the correct HitResult
	private static final double SHAPE_PADDING = 0.1;

	public BlockNode(TransportMode transportMode) {
		super(org.mtr.mod.Blocks.createDefaultBlockSettings(true).nonOpaque());
		this.transportMode = transportMode;
	}

	@Nonnull
	@Override
	public ActionResult onUse2(BlockState blockState, World world, BlockPos blockPos, PlayerEntity playerEntity, Hand hand, BlockHitResult hit) {
		if (world.isClient() && playerEntity.isHolding(Items.BRUSH.get())) {
			final ObjectObjectImmutablePair<Rail, BlockPos> railAndBlockPos = MinecraftClientData.getInstance().getFacingRailAndBlockPos(false);
			if (railAndBlockPos != null) {
				ClientPacketHelper.openRailShapeModifierScreen(railAndBlockPos.left().getHexId());
				return ActionResult.SUCCESS;
			} else if (!blockState.get(new Property<>(IS_CONNECTED.data))) {
				ClientPacketHelper.openNodeCustomAngleScreen(blockPos);
				return ActionResult.SUCCESS;
			} else {
				return ActionResult.FAIL;
			}
		} else {
			return ActionResult.FAIL;
		}
	}

	@Nonnull
	@Override
	public BlockState getPlacementState2(ItemPlacementContext itemPlacementContext) {
		final int quadrant = Angle.getQuadrant(itemPlacementContext.getPlayerYaw(), true);
		return super.getPlacementState2(itemPlacementContext)
				.with(new Property<>(FACING.data), quadrant % 8 >= 4)
				.with(new Property<>(IS_45.data), quadrant % 4 >= 2)
				.with(new Property<>(IS_22_5.data), quadrant % 2 == 1)
				.with(new Property<>(IS_CONNECTED.data), false);
	}

	@Override
	public void onBreak2(World world, BlockPos pos, BlockState state, PlayerEntity player) {
		super.onBreak2(world, pos, state, player);
		if (!world.isClient()) {
			PacketDeleteData.sendDirectlyToServerRailNodePosition(ServerWorld.cast(world), Init.blockPosToPosition(pos));
		}
	}

	@Nonnull
	@Override
	public final VoxelShape getOutlineShape2(BlockState blockState, BlockView world, BlockPos pos, ShapeContext context) {
		return Block.createCuboidShape(SHAPE_PADDING, getShapeY1(), SHAPE_PADDING, 16 - SHAPE_PADDING, getShapeY2(blockState), 16 - SHAPE_PADDING);
	}

	@Nonnull
	@Override
	public VoxelShape getCollisionShape2(BlockState state, BlockView world, BlockPos pos, ShapeContext context) {
		return VoxelShapes.empty();
	}

	@Override
	public void addBlockProperties(List<HolderBase<?>> properties) {
		super.addBlockProperties(properties);
		properties.add(FACING);
		properties.add(IS_22_5);
		properties.add(IS_45);
		properties.add(IS_CONNECTED);
	}

	@Nonnull
	@Override
	public BlockEntityExtension createBlockEntity(BlockPos blockPos, BlockState blockState) {
		return new BlockEntity(blockPos, blockState);
	}

	double getShapeY1() {
		return SHAPE_PADDING;
	}

	double getShapeY2(BlockState blockState) {
		return IBlock.getStatePropertySafe(blockState, IS_CONNECTED) ? 1 : 16 - SHAPE_PADDING;
	}

	public static void resetRailNode(ServerWorld serverWorld, BlockPos blockPos) {
		final BlockState state = serverWorld.getBlockState(blockPos);
		if (state.getBlock().data instanceof BlockNode) {
			serverWorld.setBlockState(blockPos, state.with(new Property<>(BlockNode.IS_CONNECTED.data), false));
		}
	}

	public static float getAngle(BlockState state) {
		return (IBlock.getStatePropertySafe(state, BlockNode.FACING) ? 0 : 90) + (IBlock.getStatePropertySafe(state, BlockNode.IS_22_5) ? 22.5F : 0) + (IBlock.getStatePropertySafe(state, BlockNode.IS_45) ? 45 : 0);
	}

	public static float getAngle(World world, BlockPos blockPos, BlockState state) {
		final org.mtr.mapping.holder.BlockEntity blockEntity = world.getBlockEntity(blockPos);
		if (blockEntity != null && blockEntity.data instanceof BlockEntity) {
			final BlockEntity nodeEntity = (BlockEntity) blockEntity.data;
			if (nodeEntity.hasCustomAngle()) {
				return nodeEntity.getCustomAngle();
			}
		}
		return getAngle(state);
	}

	public static float getAngle(ClientWorld world, BlockPos blockPos, BlockState state) {
		final org.mtr.mapping.holder.BlockEntity blockEntity = world.getBlockEntity(blockPos);
		if (blockEntity != null && blockEntity.data instanceof BlockEntity) {
			final BlockEntity nodeEntity = (BlockEntity) blockEntity.data;
			if (nodeEntity.hasCustomAngle()) {
				return nodeEntity.getCustomAngle();
			}
		}
		return getAngle(state);
	}

	public static boolean hasCustomAngle(World world, BlockPos blockPos) {
		final org.mtr.mapping.holder.BlockEntity blockEntity = world.getBlockEntity(blockPos);
		return blockEntity != null && blockEntity.data instanceof BlockEntity && ((BlockEntity) blockEntity.data).hasCustomAngle();
	}

	public static boolean hasCustomAngle(ClientWorld world, BlockPos blockPos) {
		final org.mtr.mapping.holder.BlockEntity blockEntity = world.getBlockEntity(blockPos);
		return blockEntity != null && blockEntity.data instanceof BlockEntity && ((BlockEntity) blockEntity.data).hasCustomAngle();
	}

	public static class BlockEntity extends BlockEntityExtension {

		private float customAngle;
		private boolean hasCustomAngle;

		private static final String KEY_CUSTOM_ANGLE = "custom_angle";
		private static final String KEY_HAS_CUSTOM_ANGLE = "has_custom_angle";

		public BlockEntity(BlockPos pos, BlockState state) {
			super(BlockEntityTypes.NODE.get(), pos, state);
		}

		@Override
		public void readCompoundTag(CompoundTag compoundTag) {
			customAngle = compoundTag.getFloat(KEY_CUSTOM_ANGLE);
			hasCustomAngle = compoundTag.getBoolean(KEY_HAS_CUSTOM_ANGLE);
		}

		@Override
		public void writeCompoundTag(CompoundTag compoundTag) {
			compoundTag.putFloat(KEY_CUSTOM_ANGLE, customAngle);
			compoundTag.putBoolean(KEY_HAS_CUSTOM_ANGLE, hasCustomAngle);
		}

		public float getCustomAngle() {
			return customAngle;
		}

		public boolean hasCustomAngle() {
			return hasCustomAngle;
		}

		public void setCustomAngle(float customAngle) {
			float normalized = customAngle % 360;
			if (normalized < 0) {
				normalized += 360;
			}
			this.customAngle = Math.round(normalized * 1000F) / 1000F;
			hasCustomAngle = true;
			markDirty2();
		}
	}

	public static class BlockContinuousMovementNode extends BlockNode {

		public final boolean upper;
		public final boolean isStation;

		public BlockContinuousMovementNode(boolean upper, boolean isStation) {
			super(TransportMode.CABLE_CAR);
			this.upper = upper;
			this.isStation = isStation;
		}

		@Nonnull
		@Override
		public BlockState getPlacementState2(ItemPlacementContext itemPlacementContext) {
			final int quadrant = Angle.getQuadrant(itemPlacementContext.getPlayerYaw(), false);
			return super.getPlacementState2(itemPlacementContext)
					.with(new Property<>(FACING.data), quadrant % 4 >= 2)
					.with(new Property<>(IS_45.data), quadrant % 2 == 1)
					.with(new Property<>(IS_22_5.data), false)
					.with(new Property<>(IS_CONNECTED.data), false);
		}

		@Override
		public void addTooltips(ItemStack stack, @Nullable BlockView world, List<MutableText> tooltip, TooltipContext options) {
			final String[] strings = (isStation ? TranslationProvider.TOOLTIP_MTR_CABLE_CAR_NODE_STATION : TranslationProvider.TOOLTIP_MTR_CABLE_CAR_NODE).getString().split("\n");
			for (final String string : strings) {
				tooltip.add(TextHelper.literal(string).formatted(TextFormatting.GRAY));
			}
		}

		@Override
		double getShapeY1() {
			return upper ? 8 : SHAPE_PADDING;
		}

		@Override
		double getShapeY2(BlockState blockState) {
			return upper ? 16 - SHAPE_PADDING : 8;
		}
	}
}
