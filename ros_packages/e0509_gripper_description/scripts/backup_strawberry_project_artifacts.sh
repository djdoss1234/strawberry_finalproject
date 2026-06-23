#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 BACKUP_DIR"
  echo "Example: $0 /media/user/USB/strawberry_project_backup_$(date +%Y%m%d)"
  exit 2
fi

DEST="$1"
STAMP="$(date +%Y%m%d_%H%M%S)"
ROOT="$DEST/strawberry_harvest_backup_$STAMP"

mkdir -p "$ROOT"/{repos,assets,media,logs,reports,manifests}

echo "[backup] destination: $ROOT"

copy_dir() {
  local src="$1"
  local dst="$2"
  if [[ -e "$src" ]]; then
    echo "[backup] rsync $src -> $dst"
    rsync -a --info=progress2 \
      --exclude build \
      --exclude install \
      --exclude log \
      --exclude __pycache__ \
      --exclude .pytest_cache \
      "$src" "$dst"
  else
    echo "[backup] skip missing: $src"
  fi
}

copy_file() {
  local src="$1"
  local dst="$2"
  if [[ -f "$src" ]]; then
    mkdir -p "$dst"
    echo "[backup] copy $src -> $dst"
    cp -a "$src" "$dst/"
  fi
}

# 1. Source repositories and runtime configs.
copy_dir /home/user/doosan_ws/src/strawberry_finalproject "$ROOT/repos/"
copy_dir /home/user/doosan_ws/src/e0509_gripper_description_legacy_miniproject_20260618 "$ROOT/repos/"
copy_dir /home/user/doosan_ws/src/dsr_gripper_tcp "$ROOT/repos/"
copy_dir /home/user/doosan_ws/src/dsr_gripper_tcp_interfaces "$ROOT/repos/"
copy_dir /home/user/doosan_ws/src/RH-P12-RN-A "$ROOT/repos/"
copy_dir /home/user/doosan_ws/src/e0509_gripper_moveit_config "$ROOT/repos/"

# 2. Tray localization and local utility data.
copy_dir /home/user/Downloads/share_tray "$ROOT/assets/"

# 3. Project-related images/videos/PPT/PDF in Downloads and Pictures.
find /home/user/Downloads /home/user/Pictures -maxdepth 2 -type f \
  \( -iname '*.mp4' -o -iname '*.mov' -o -iname '*.avi' -o -iname '*.mkv' \
     -o -iname '*.gif' -o -iname '*.png' -o -iname '*.jpg' -o -iname '*.jpeg' \
     -o -iname '*.pptx' -o -iname '*.pdf' -o -iname '*.svg' \) \
  | grep -Ei 'strawberry|딸기|harvest|수확|doosan|gripper|tray|fusion|motion|IMG_|Screenshot|제목|스마트|로봇|PPT|ppt' \
  | while read -r f; do
      rel="${f#/home/user/}"
      mkdir -p "$ROOT/media/$(dirname "$rel")"
      cp -a "$f" "$ROOT/media/$rel"
    done

# 4. Lightweight manifests for future portfolio reconstruction.
{
  echo "# Strawberry Harvest Backup Manifest"
  echo "created_at=$STAMP"
  echo "host=$(hostname)"
  echo
  echo "## Git status"
  for repo in \
    /home/user/doosan_ws/src/strawberry_finalproject \
    /home/user/doosan_ws/src/e0509_gripper_description_legacy_miniproject_20260618 \
    /home/user/doosan_ws/src/dsr_gripper_tcp; do
    if [[ -d "$repo/.git" ]]; then
      echo "### $repo"
      git -C "$repo" status --short --branch || true
      git -C "$repo" log --oneline -20 || true
      echo
    fi
  done
} > "$ROOT/manifests/README_BACKUP_MANIFEST.md"

find "$ROOT" -type f | sort > "$ROOT/manifests/all_files.txt"
du -sh "$ROOT" > "$ROOT/manifests/backup_size.txt"

echo "[backup] done"
echo "[backup] root: $ROOT"
echo "[backup] manifest: $ROOT/manifests/README_BACKUP_MANIFEST.md"
