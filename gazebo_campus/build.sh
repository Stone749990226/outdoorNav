#!/usr/bin/env bash
set -eo pipefail
campus_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(dirname "$campus_dir")"
if [[ -x "$repo_dir/.venv/bin/python" ]]; then
  uv run --python "$repo_dir/.venv/bin/python" python "$campus_dir/scripts/build_scene.py" "$@"
elif [[ -x "$repo_dir/venv/bin/python" ]]; then
  uv run --python "$repo_dir/venv/bin/python" python "$campus_dir/scripts/build_scene.py" "$@"
else
  conda run --no-capture-output -n torch python "$campus_dir/scripts/build_scene.py" "$@"
fi

# 生成之后补法线（2026-10-12 加）—— 没法线的 mesh 在 Fortress 里渲染成纯白且不受光
scene_dir="$campus_dir/generated/models/campus_scene"
for m in roads plaza; do
  f="$scene_dir/meshes/$m.dae"
  [ -f "$f" ] || continue
  # 注意 add_dae_normals.py 的退出码语义：0 = 已补好；1 = 已经有法线、拒绝重复注入
  # 它不是校验器，别写成 --check 加与号那种用法 —— 第二次生成会在那儿挂掉
  if grep -q 'semantic="NORMAL"' "$f"; then
    echo "  $m.dae 已有法线，跳过"
  elif python3 "$campus_dir/scripts/add_dae_normals.py" "$f" "$f.tmp"; then
    mv "$f.tmp" "$f"
  else
    echo "补法线失败: $m.dae" >&2; exit 1
  fi
done
