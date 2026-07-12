#!/bin/bash

mkdir -p weights

declare -A sim_models=(
    ["gate"]="https://drive.google.com/file/d/1811hLDDxDxHMyztwxl88FgQLGjd98Rpu/view?usp=sharing"
    ["slalom"]="https://drive.google.com/file/d/1fpKPookhjDVDDNKeEgshl0-yfyH-V0jp/view?usp=sharing"
    ["torp"]="https://drive.google.com/file/d/10ptY-xR_U4myQR5u6esSzd5Hgt2FNk6H/view?usp=sharing"
)

declare -A models=(
    ["gate"]=""
    ["slalom"]=""
    ["torp"]=""
)

if [[ " $* " =~ " --sim " ]]; then
    for model in "${!sim_models[@]}"; do
        gdown "${sim_models[$model]}" -O "weights/$model.pt"
    done
else
    for model in "${!models[@]}"; do
        gdown "${models[$model]}" -O "weights/$model.pt"
    done
fi
