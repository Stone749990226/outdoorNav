"""Fetch every sub-repo listed in tools/sources.json at its branch tip.

Missing repos are shallow-cloned at the configured branch; existing repos are
reset to that branch's newest commit, so local uncommitted changes are
overwritten. An empty branch in sources.json means "follow the remote default
branch". Stdlib only.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCES_JSON = ROOT / "tools" / "sources.json"


def load_repos() -> list[dict]:
    data = json.loads(SOURCES_JSON.read_text(encoding="utf-8"))
    return list(data["repos"])


def run(cmd: list[str], cwd: Path | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)


def remote_default_branch(url: str) -> str:
    """Resolve the remote's default branch via ls-remote."""
    proc = run(["git", "ls-remote", "--symref", url, "HEAD"])
    for line in proc.stdout.splitlines():
        if line.startswith("ref: refs/heads/"):
            return line.split("\t", 1)[1].strip()
    raise SystemExit(f"无法获取 {url} 的默认分支：{proc.stderr.strip()}")


def head_sha(path: Path) -> str:
    proc = run(["git", "-C", str(path), "rev-parse", "--short", "HEAD"])
    return proc.stdout.strip()


def clone_repo(repo: dict, path: Path, branch: str) -> bool:
    cmd = ["git", "clone", "--depth", "1"]
    if branch:
        cmd += ["-b", branch]
    cmd += [repo["url"], str(path)]
    proc = run(cmd)
    if proc.returncode != 0:
        print(f"  失败：{proc.stderr.strip()}")
        return False
    print(f"  完成：{head_sha(path)}")
    return True


def update_repo(path: Path, branch: str) -> bool:
    """Fetch the branch tip and hard-reset the working tree onto it."""
    fetch = run(["git", "-C", str(path), "fetch", "--depth", "1", "origin", branch])
    if fetch.returncode != 0:
        print(f"  失败：{fetch.stderr.strip()}")
        return False
    reset = run(["git", "-C", str(path), "reset", "--hard", "FETCH_HEAD"])
    if reset.returncode != 0:
        print(f"  失败：{reset.stderr.strip()}")
        return False
    checkout = run(["git", "-C", str(path), "checkout", "-B", branch])
    if checkout.returncode != 0:
        print(f"  失败：{checkout.stderr.strip()}")
        return False
    print(f"  完成：{head_sha(path)}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="拉取/更新 tools/sources.json 里各子仓库分支的最新代码"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="只预览将执行的操作，不联网"
    )
    args = parser.parse_args()

    repos = load_repos()
    failed: list[str] = []
    for repo in repos:
        name = repo["name"]
        path = ROOT / name
        branch = repo.get("branch", "")
        label = branch or "(远程默认分支)"

        if not path.exists():
            print(f"[clone] {name} ← {repo['url']} ({label})")
            if args.dry_run:
                continue
            if not clone_repo(repo, path, branch):
                failed.append(name)
        elif not (path / ".git").exists():
            print(f"[跳过] {name}：目录已存在但不是 git 仓库，请人工处理")
        else:
            print(f"[更新] {name} → {label}")
            if args.dry_run:
                continue
            resolved = branch or remote_default_branch(repo["url"])
            if not update_repo(path, resolved):
                failed.append(name)

    if args.dry_run:
        print("\n以上为预览，未做任何修改。")
        return 0
    if failed:
        print(f"\n失败 {len(failed)} 个：{', '.join(failed)}")
        return 1
    print(f"\n全部 {len(repos)} 个仓库已是最新。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
