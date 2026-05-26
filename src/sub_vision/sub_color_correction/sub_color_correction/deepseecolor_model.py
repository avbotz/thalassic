# Copyright 2023 Stewart Jamieson, Woods Hole Oceanographic Institution
# Adapted from warplab/DeepSeeColor for ROS 2 streaming use.
# DeepSeeColor is licensed under AGPL-3.0-only.

from __future__ import annotations

from dataclasses import dataclass

import torch
import torch.nn as nn
import torch.nn.functional as F

try:
    import kornia.morphology as morph
except ImportError:  # pragma: no cover - node validates this at runtime.
    morph = None


class BackscatterNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.backscatter_conv = nn.Conv2d(1, 3, 1, bias=False)
        self.residual_conv = nn.Conv2d(1, 3, 1, bias=False)
        nn.init.uniform_(self.backscatter_conv.weight, 0, 5)
        nn.init.uniform_(self.residual_conv.weight, 0, 5)
        self.B_inf = nn.Parameter(torch.rand(3, 1, 1))
        self.J_prime = nn.Parameter(torch.rand(3, 1, 1))
        self.sigmoid = nn.Sigmoid()
        self.relu = nn.ReLU()

    def forward(self, image, depth):
        beta_b_conv = self.relu(self.backscatter_conv(depth))
        beta_d_conv = self.relu(self.residual_conv(depth))
        Bc = self.B_inf * (1 - torch.exp(-beta_b_conv)) + self.J_prime * torch.exp(-beta_d_conv)
        backscatter = self.sigmoid(Bc)
        backscatter_masked = backscatter * (depth > 0.0).repeat(1, 3, 1, 1)
        direct = image - backscatter_masked
        return direct, backscatter


class DeattenuateNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.attenuation_conv = nn.Conv2d(1, 6, 1, bias=False)
        nn.init.uniform_(self.attenuation_conv.weight, 0, 5)
        self.attenuation_coef = nn.Parameter(torch.rand(6, 1, 1))
        self.relu = nn.ReLU()
        self.wb = nn.Parameter(torch.rand(1, 1, 1))
        nn.init.constant_(self.wb, 1)

    def forward(self, direct, depth):
        attn_conv = torch.exp(-self.relu(self.attenuation_conv(depth)))
        beta_d = torch.stack(
            tuple(
                torch.sum(
                    attn_conv[:, i : i + 2, :, :] * self.relu(self.attenuation_coef[i : i + 2]),
                    dim=1,
                )
                for i in range(0, 6, 2)
            ),
            dim=1,
        )
        clamp_max = float(torch.log(torch.tensor([3.0], device=depth.device)))
        f = torch.exp(torch.clamp(beta_d * depth, 0, clamp_max))
        f_masked = f * ((depth == 0.0) / f + (depth > 0.0))
        J = f_masked * direct * self.wb
        J[torch.isnan(J)] = 0
        return f_masked, J


class BackscatterLoss(nn.Module):
    def __init__(self, cost_ratio=1000.0):
        super().__init__()
        self.l1 = nn.L1Loss()
        self.smooth_l1 = nn.SmoothL1Loss(beta=0.2)
        self.relu = nn.ReLU()
        self.cost_ratio = cost_ratio

    def forward(self, direct):
        pos = self.l1(self.relu(direct), torch.zeros_like(direct))
        neg = self.smooth_l1(self.relu(-direct), torch.zeros_like(direct))
        return self.cost_ratio * neg + pos


class DeattenuateLoss(nn.Module):
    def __init__(self):
        super().__init__()
        self.mse = nn.MSELoss()
        self.relu = nn.ReLU()
        self.target_intensity = 0.5

    def forward(self, direct, J):
        saturation_loss = (self.relu(-J) + self.relu(J - 1)).square().mean()
        init_spatial = torch.std(direct, dim=[2, 3])
        channel_intensities = torch.mean(J, dim=[2, 3], keepdim=True)
        channel_spatial = torch.std(J, dim=[2, 3])
        intensity_loss = (channel_intensities - self.target_intensity).square().mean()
        spatial_variation_loss = self.mse(channel_spatial, init_spatial)
        return saturation_loss + intensity_loss + spatial_variation_loss


@dataclass
class DeepSeeColorStats:
    backscatter_loss: float
    deattenuation_loss: float
    trained_iterations: int


class DeepSeeColorProcessor:
    def __init__(
        self,
        device: str,
        init_iters: int,
        iters: int,
        learning_rate: float,
        depth_quantile: float,
        mask_max_depth: bool,
    ):
        self.device = torch.device(device)
        self.init_iters = init_iters
        self.iters = iters
        self.depth_quantile = depth_quantile
        self.mask_max_depth = mask_max_depth
        self.is_initialized = False

        self.bs_model = BackscatterNet().to(self.device)
        self.da_model = DeattenuateNet().to(self.device)
        self.bs_criterion = BackscatterLoss().to(self.device)
        self.da_criterion = DeattenuateLoss().to(self.device)
        self.bs_optimizer = torch.optim.Adam(self.bs_model.parameters(), lr=learning_rate)
        self.da_optimizer = torch.optim.Adam(self.da_model.parameters(), lr=learning_rate)
        self.kernel = torch.ones(3, 3, device=self.device)

    def correct(self, rgb, depth, max_dimension: int = 0) -> tuple[torch.Tensor, DeepSeeColorStats]:
        original_size = rgb.shape[-2:]
        rgb, depth = self._resize_for_inference(rgb, depth, max_dimension)
        depth = self._prepare_depth(depth)

        train_iters = self.init_iters if not self.is_initialized else self.iters
        train_iters = max(0, train_iters)

        direct = torch.clamp(rgb, 0.0, 1.0)
        backscatter_loss = torch.tensor(0.0, device=self.device)
        deattenuation_loss = torch.tensor(0.0, device=self.device)

        for _ in range(train_iters):
            direct, _ = self.bs_model(rgb, depth)
            backscatter_loss = self.bs_criterion(direct)
            self.bs_optimizer.zero_grad()
            backscatter_loss.backward()
            self.bs_optimizer.step()

        if train_iters == 0:
            direct, _ = self.bs_model(rgb, depth)

        direct_no_grad = self._normalize_direct(direct).detach()

        for _ in range(train_iters):
            _, corrected = self.da_model(direct_no_grad, depth)
            deattenuation_loss = self.da_criterion(direct_no_grad, corrected)
            self.da_optimizer.zero_grad()
            deattenuation_loss.backward()
            self.da_optimizer.step()

        if train_iters == 0:
            _, corrected = self.da_model(direct_no_grad, depth)

        self.is_initialized = True
        corrected = torch.clamp(corrected.detach(), 0.0, 1.0)
        if corrected.shape[-2:] != original_size:
            corrected = F.interpolate(corrected, size=original_size, mode="bilinear", align_corners=False)

        return corrected, DeepSeeColorStats(
            backscatter_loss=float(backscatter_loss.detach().cpu()),
            deattenuation_loss=float(deattenuation_loss.detach().cpu()),
            trained_iterations=train_iters,
        )

    def _resize_for_inference(self, rgb, depth, max_dimension: int):
        if max_dimension <= 0:
            return rgb, depth

        height, width = rgb.shape[-2:]
        longest = max(height, width)
        if longest <= max_dimension:
            return rgb, depth

        scale = max_dimension / float(longest)
        size = (max(1, int(round(height * scale))), max(1, int(round(width * scale))))
        rgb = F.interpolate(rgb, size=size, mode="bilinear", align_corners=False)
        depth = F.interpolate(depth, size=size, mode="nearest")
        return rgb, depth

    def _prepare_depth(self, depth):
        depth = depth.clone()
        depth[~torch.isfinite(depth)] = 0.0
        depth[depth < 0.0] = 0.0

        if self.mask_max_depth and torch.any(depth > 0.0):
            depth[depth == 0.0] = torch.max(depth)

        valid = depth[depth > 0.0]
        if valid.numel() > 0 and self.depth_quantile > 0.0:
            low = torch.quantile(valid, self.depth_quantile)
            high = torch.quantile(valid, 1.0 - self.depth_quantile)
            depth[(depth < low) | (depth > high)] = 0.0

        if morph is not None:
            depth = morph.closing(depth, self.kernel)

        return depth

    def _normalize_direct(self, direct):
        direct_mean = direct.mean(dim=[2, 3], keepdim=True)
        direct_std = direct.std(dim=[2, 3], keepdim=True).clamp_min(1e-6)
        direct_z = (direct - direct_mean) / direct_std
        clamped_z = torch.clamp(direct_z, -5, 5)
        min_mean = torch.tensor([1.0 / 255.0], device=self.device)
        return torch.clamp((clamped_z * direct_std) + torch.maximum(direct_mean, min_mean), 0, 1)
