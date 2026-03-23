#!/usr/bin/env python3
"""Install ROS deb packages into a local overlay without sudo.

This script resolves package URIs via `apt-get --print-uris install`,
downloads the required .deb files, extracts them into a local root,
and writes an `activate.bash` helper for using the overlay.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import textwrap
import urllib.request
from pathlib import Path


URI_RE = re.compile(r"^'(?P<url>[^']+)' (?P<filename>\S+) (?P<size>\d+) ")
GZ_LIBRARY_PATH_RE = re.compile(
    r"(^\s*library_path:\s+)(?P<path>/opt/ros/jazzy[^\s#]+)",
    re.MULTILINE,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        default=".cache/ros_overlay",
        help="Overlay root directory (default: %(default)s)",
    )
    parser.add_argument(
        "--cache-dir",
        default=".cache/ros_overlay/debs",
        help="Downloaded .deb cache directory (default: %(default)s)",
    )
    parser.add_argument(
        "packages",
        nargs="+",
        help="APT package names to install into the overlay",
    )
    return parser.parse_args()


def resolve_package_uris(packages: list[str]) -> list[tuple[str, str, int]]:
    cmd = ["apt-get", "-qq", "--print-uris", "install", *packages]
    result = subprocess.run(cmd, check=True, capture_output=True, text=True)

    uris: list[tuple[str, str, int]] = []
    for line in result.stdout.splitlines():
        match = URI_RE.match(line.strip())
        if not match:
            continue
        uris.append(
            (
                match.group("url"),
                match.group("filename"),
                int(match.group("size")),
            )
        )
    if not uris:
        raise RuntimeError("No package URIs resolved. Check package names or apt sources.")
    return uris


def download_file(url: str, dest: Path, expected_size: int) -> None:
    if dest.exists() and dest.stat().st_size == expected_size:
        return
    with urllib.request.urlopen(url) as response, open(dest, "wb") as out:
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            out.write(chunk)


def extract_deb(deb_path: Path, root: Path) -> None:
    subprocess.run(
        ["dpkg-deb", "-x", str(deb_path), str(root)],
        check=True,
        capture_output=True,
        text=True,
    )


def relocate_gz_vendor_paths(root: Path) -> None:
    overlay_prefix = (root / "opt/ros/jazzy").resolve()
    overlay_root = str(root.resolve())
    duplicated_overlay_root = overlay_root + overlay_root.lstrip("/")
    for yaml_path in (overlay_prefix / "opt").glob("*/share/gz/*.yaml"):
        text = yaml_path.read_text()
        while duplicated_overlay_root in text:
            text = text.replace(duplicated_overlay_root, overlay_root)
        relocated = GZ_LIBRARY_PATH_RE.sub(
            lambda match: (
                f"{match.group(1)}"
                f"{match.group('path').replace('/opt/ros/jazzy', str(overlay_prefix), 1)}"
            ),
            text,
        )
        if relocated != text:
            yaml_path.write_text(relocated)


def write_activate_script(root: Path) -> None:
    activate_path = root / "activate.bash"
    activate_path.write_text(
        textwrap.dedent(
            """\
            #!/usr/bin/env bash
            _overlay_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
            _overlay_prefix="$_overlay_root/opt/ros/jazzy"
            if [ -f /opt/ros/jazzy/setup.bash ]; then
              source /opt/ros/jazzy/setup.bash
            fi

            # Prefer the ROS/apt Python environment over user site-packages.
            export PYTHONNOUSERSITE=1
            export PATH="$_overlay_prefix/bin:$_overlay_root/usr/bin:${PATH:-}"
            export PYTHONPATH="$_overlay_prefix/lib/python3.12/site-packages:$_overlay_root/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
            export LD_LIBRARY_PATH="$_overlay_prefix/opt/gz_ogre_next_vendor/lib/OGRE-Next:$_overlay_prefix/lib:$_overlay_prefix/lib/x86_64-linux-gnu:$_overlay_root/usr/lib:$_overlay_root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
            export CMAKE_PREFIX_PATH="$_overlay_prefix:${CMAKE_PREFIX_PATH:-}"
            export COLCON_PREFIX_PATH="$_overlay_prefix:${COLCON_PREFIX_PATH:-}"
            export AMENT_PREFIX_PATH="$_overlay_prefix:${AMENT_PREFIX_PATH:-}"
            export GZ_SIM_RESOURCE_PATH="$_overlay_prefix/share:${GZ_SIM_RESOURCE_PATH:-}"
            export RUBYLIB="$_overlay_root/usr/lib/ruby/vendor_ruby:$_overlay_root/usr/lib/ruby/3.2.0:$_overlay_root/usr/lib/x86_64-linux-gnu/ruby/vendor_ruby/3.2.0:$_overlay_root/usr/lib/x86_64-linux-gnu/ruby/3.2.0:${RUBYLIB:-}"
            export GZ_SIM_SYSTEM_PLUGIN_PATH="$_overlay_prefix/opt/gz_sim_vendor/lib/gz-sim-8/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
            export GZ_GUI_PLUGIN_PATH="$_overlay_prefix/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui:${GZ_GUI_PLUGIN_PATH:-}"
            export GZ_SIM_SERVER_CONFIG_PATH="$_overlay_prefix/opt/gz_sim_vendor/share/gz/gz-sim8/server.config"
            export GZ_SIM_PHYSICS_ENGINE_PATH="$_overlay_prefix/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins:${GZ_SIM_PHYSICS_ENGINE_PATH:-}"
            export GZ_SIM_RENDER_ENGINE_PATH="$_overlay_prefix/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins:${GZ_SIM_RENDER_ENGINE_PATH:-}"
            export GZ_RENDERING_PLUGIN_PATH="$_overlay_prefix/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins:${GZ_RENDERING_PLUGIN_PATH:-}"
            export GZ_RENDERING_RESOURCE_PATH="$_overlay_prefix/opt/gz_rendering_vendor/share/gz/gz-rendering8"
            export OGRE2_RESOURCE_PATH="$_overlay_prefix/opt/gz_ogre_next_vendor/lib/OGRE-Next"

            for _gz_model_dir in "$_overlay_prefix"/share/*/models; do
              if [ -d "$_gz_model_dir" ]; then
                export GZ_SIM_RESOURCE_PATH="$_gz_model_dir:${GZ_SIM_RESOURCE_PATH:-}"
              fi
            done
            unset _gz_model_dir

            for _gz_dir in "$_overlay_prefix"/opt/*/share/gz; do
              if [ -d "$_gz_dir" ]; then
                export GZ_CONFIG_PATH="$_gz_dir:${GZ_CONFIG_PATH:-}"
              fi
            done
            unset _gz_dir

            if [ -d "$_overlay_prefix/share" ]; then
              for _pkg_setup in "$_overlay_prefix"/share/*/local_setup.sh; do
                if [ -f "$_pkg_setup" ]; then
                  AMENT_CURRENT_PREFIX="$_overlay_prefix" . "$_pkg_setup"
                fi
              done
              unset _pkg_setup
            fi

            echo "ROS overlay active: $_overlay_root"
            """
        )
    )
    activate_path.chmod(0o755)

    xvfb_helper_path = root / "run_with_xvfb.bash"
    xvfb_helper_path.write_text(
        textwrap.dedent(
            """\
            #!/usr/bin/env bash
            set -eo pipefail
            _overlay_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
            source "$_overlay_root/activate.bash" >/dev/null

            if ! command -v xvfb-run >/dev/null 2>&1; then
              echo "xvfb-run not found in overlay PATH" >&2
              exit 1
            fi
            if [ "$#" -eq 0 ]; then
              echo "usage: $(basename "$0") <command> [args...]" >&2
              exit 2
            fi

            export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
            exec xvfb-run -a -s "${XVFB_SCREEN_ARGS:--screen 0 1280x1024x24}" "$@"
            """
        )
    )
    xvfb_helper_path.chmod(0o755)


def main() -> int:
    args = parse_args()
    root = Path(args.root).resolve()
    cache_dir = Path(args.cache_dir).resolve()
    root.mkdir(parents=True, exist_ok=True)
    cache_dir.mkdir(parents=True, exist_ok=True)

    uris = resolve_package_uris(args.packages)
    total_bytes = sum(size for _, _, size in uris)
    print(f"Resolving {len(uris)} debs, total {total_bytes / (1024 ** 2):.1f} MiB")

    for idx, (url, filename, size) in enumerate(uris, start=1):
        deb_path = cache_dir / filename
        print(f"[{idx}/{len(uris)}] download {filename}")
        download_file(url, deb_path, size)
        extract_deb(deb_path, root)

    relocate_gz_vendor_paths(root)
    write_activate_script(root)
    print(f"Overlay installed at: {root}")
    print(f"Activate with: source {root / 'activate.bash'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
