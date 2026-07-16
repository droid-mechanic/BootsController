#!/usr/bin/env bash
# install_kiosk.sh
# ════════════════
# Phase 9 — One-shot kiosk setup script.
#
# Run from the project root on the Raspberry Pi:
#   chmod +x kiosk/install_kiosk.sh
#   sudo kiosk/install_kiosk.sh
#
# What it does:
#   1. Installs required system packages
#   2. Installs the rc-controller systemd service (backend)
#   3. Installs the rc-kiosk systemd service (Chromium)
#   4. Installs autologin drop-in for tty1
#   5. Appends X auto-start to /home/pi/.bash_profile
#   6. Installs ~/.xinitrc
#   7. Adds pi to 'input' and 'dialout' groups (evdev + serial)
#   8. Enables both services

set -euo pipefail

# ── Must run as root ──────────────────────────────────────────────────────
if [[ "$EUID" -ne 0 ]]; then
    echo "Run with sudo: sudo $0"
    exit 1
fi

# ── Resolve project directory (script lives in kiosk/ subdirectory) ───────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PI_HOME="/home/pi"

echo "=== RC Controller Kiosk Installer ==="
echo "Project dir : $PROJECT_DIR"

# ── 1. System packages ────────────────────────────────────────────────────
echo ""
echo "[1/8] Installing system packages..."
apt-get update -qq
apt-get install -y --no-install-recommends \
    xorg \
    openbox \
    chromium-browser \
    unclutter \
    curl \
    python3-pip

# Python deps

python3 -m venv /home/pi/rc-controller/.venv
/home/pi/rc-controller/.venv/bin/pip install evdev pyserial flask flask-sock

echo "      Packages installed."

# ── 2. Patch WorkingDirectory in service file ─────────────────────────────
echo ""
echo "[2/8] Configuring service WorkingDirectory..."
sed -i "s|WorkingDirectory=.*|WorkingDirectory=$PROJECT_DIR|" \
    "$SCRIPT_DIR/rc-controller.service"

# ── 3. Install rc-controller.service ─────────────────────────────────────
echo ""
echo "[3/8] Installing rc-controller.service..."
cp "$SCRIPT_DIR/rc-controller.service" /etc/systemd/system/rc-controller.service

# ── 4. Install rc-kiosk.service ──────────────────────────────────────────
echo ""
echo "[4/8] Installing rc-kiosk.service..."
cp "$SCRIPT_DIR/rc-kiosk.service" /etc/systemd/system/rc-kiosk.service

# ── 5. Autologin drop-in ──────────────────────────────────────────────────
echo ""
echo "[5/8] Installing autologin drop-in..."
mkdir -p /etc/systemd/system/getty@tty1.service.d/
cp "$SCRIPT_DIR/autologin.conf" \
    /etc/systemd/system/getty@tty1.service.d/autologin.conf

# ── 6. ~/.bash_profile X auto-start ──────────────────────────────────────
echo ""
echo "[6/8] Configuring ~/.bash_profile..."
MARKER="# rc-kiosk: auto-start X"
if ! grep -qF "$MARKER" "$PI_HOME/.bash_profile" 2>/dev/null; then
    echo "" >> "$PI_HOME/.bash_profile"
    echo "$MARKER" >> "$PI_HOME/.bash_profile"
    cat "$SCRIPT_DIR/bash_profile_append" >> "$PI_HOME/.bash_profile"
    chown pi:pi "$PI_HOME/.bash_profile"
    echo "      Appended to ~/.bash_profile."
else
    echo "      ~/.bash_profile already configured — skipping."
fi

# ── 7. ~/.xinitrc ─────────────────────────────────────────────────────────
echo ""
echo "[7/8] Installing ~/.xinitrc..."
cp "$SCRIPT_DIR/xinitrc" "$PI_HOME/.xinitrc"
chmod +x "$PI_HOME/.xinitrc"
chown pi:pi "$PI_HOME/.xinitrc"

# ── 8. Group memberships ──────────────────────────────────────────────────
echo ""
echo "[8/8] Adding pi to input and dialout groups..."
usermod -aG input   pi
usermod -aG dialout pi
echo "      (Group changes take effect on next login / reboot)"

# ── Enable and reload ─────────────────────────────────────────────────────
echo ""
echo "Reloading systemd and enabling services..."
systemctl daemon-reload
systemctl enable rc-controller.service
systemctl enable rc-kiosk.service

echo ""
echo "════════════════════════════════════════"
echo "  Installation complete."
echo ""
echo "  Reboot to enter kiosk mode:"
echo "    sudo reboot"
echo ""
echo "  Monitor logs:"
echo "    journalctl -u rc-controller -f"
echo "    journalctl -u rc-kiosk -f"
echo ""
echo "  Stop kiosk temporarily:"
echo "    sudo systemctl stop rc-kiosk"
echo "════════════════════════════════════════"
