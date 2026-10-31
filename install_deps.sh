#!/usr/bin/env bash
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
#
# Installs Gazebo Harmonic and the tools this repository needs on Ubuntu 24.04.
# Run it from the repository folder: ./install_deps.sh
set -euo pipefail

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "24.04" ]]; then
  echo "This script supports Ubuntu 24.04 only." >&2
  exit 1
fi

SUDO=""
[[ "$(id -u)" -ne 0 ]] && SUDO="sudo"

echo "==> Gazebo package repository"
$SUDO apt-get update
$SUDO apt-get install -y curl lsb-release gnupg
$SUDO curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | $SUDO tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
$SUDO apt-get update

echo "==> Gazebo Harmonic, compiler and Python packages"
$SUDO apt-get install -y gz-harmonic build-essential cmake \
  python3-gz-transport13 python3-gz-msgs10 python3-numpy python3-scipy python3-matplotlib python3-tk

echo
echo "Done. Next step: the 'Build' section of README.md."
