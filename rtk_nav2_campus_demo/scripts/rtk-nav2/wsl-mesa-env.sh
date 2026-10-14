#!/usr/bin/env bash
# Source this file to use the optional Mesa installation for this process only.
# WSL_MESA_PREFIX=system leaves the system graphics libraries selected.
wsl_mesa_prefix="${WSL_MESA_PREFIX:-${HOME}/.local/share/outdoornav/mesa-25.0.7}"
if [[ "${wsl_mesa_prefix}" == system ]]; then
    return 0
fi
if [[ ! -d "${wsl_mesa_prefix}" && -z "${WSL_MESA_PREFIX:-}" ]]; then
    return 0
fi
wsl_mesa_lib="${wsl_mesa_prefix}/usr/lib/x86_64-linux-gnu"
if [[ ! -f "${wsl_mesa_lib}/libGLX_mesa.so.0" ||
      ! -f "${wsl_mesa_lib}/dri/d3d12_dri.so" ||
      ! -f "${wsl_mesa_prefix}/usr/share/glvnd/egl_vendor.d/50_mesa.json" ]]; then
    echo "Incomplete Mesa installation: ${wsl_mesa_prefix}" >&2
    return 1
fi
export LD_LIBRARY_PATH="${wsl_mesa_lib}:/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export LIBGL_DRIVERS_PATH="${wsl_mesa_lib}/dri"
export __EGL_VENDOR_LIBRARY_FILENAMES="${wsl_mesa_prefix}/usr/share/glvnd/egl_vendor.d/50_mesa.json"
# Mesa 25's generic DRI loader needs an explicit D3D12 selection on WSLg.
export GALLIUM_DRIVER=d3d12
export WSL_MESA_ACTIVE_PREFIX="${wsl_mesa_prefix}"
