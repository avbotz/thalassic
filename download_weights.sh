#!/bin/bash

mkdir -p weights

declare -A sim_models=(
    ["gate"]="https://drive.google.com/file/d/1811hLDDxDxHMyztwxl88FgQLGjd98Rpu/view"
    ["slalom"]="https://drive.google.com/file/d/1fpKPookhjDVDDNKeEgshl0-yfyH-V0jp/view"
    ["torp"]="https://drive.google.com/file/d/10ptY-xR_U4myQR5u6esSzd5Hgt2FNk6H/view"
    ["bins_blood"]="https://drive.google.com/file/d/1Ddo-M3zvYALfrasv8MyDgGH87v2yXbon/view"
    ["bins_fire"]="https://drive.google.com/file/d/1Ls-4B011pH3gqthPT8Ppgnmjf74c1qOn/view"
    ["path_marker"]="https://drive.google.com/file/d/1Z48v2QMXTAKXQeYasLrr7ppSr43AZk7q/view"
    ["slalom"]="https://drive.google.com/file/d/1fpKPookhjDVDDNKeEgshl0-yfyH-V0jp/view"
    ["octagon_gate_images"]="https://drive.google.com/drive/u/0/folders/1bESRgUH0ChXFrFy241ReqvT3KSzyoTIK"
)

declare -A models=(
    ["gate"]=""
    ["slalom"]=""
    ["torp"]=""
    ["bins_blood"]=""
    ["bins_fire"]=""
    ["path_marker"]=""
    ["slalom"]=""
    ["octagon_gate_images"]=""
)

if [[ " $* " =~ " --sim " ]]; then
    for model in "${!sim_models[@]}"; do
        gdown "${sim_models[$model]}" -O "weights/$model.onnx"
    done
else
    for model in "${!models[@]}"; do
        gdown "${models[$model]}" -O "weights/$model.pt"
    done
fi
