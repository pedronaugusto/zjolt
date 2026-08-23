#!/usr/bin/env bash
#
# Installs a pre-push hook that runs ci/run.sh.
#
# Optional, and worth it: it catches locally what CI would otherwise report
# later on a pull request. Run once per clone.
#
#   ci/install-hooks.sh
#
# To push without running it (a docs-only change, say):
#
#   git push --no-verify

set -euo pipefail
cd "$(dirname "$0")/.."

hooks_dir=$(git rev-parse --git-path hooks)
mkdir -p "$hooks_dir"

cat > "$hooks_dir/pre-push" <<'HOOK'
#!/usr/bin/env bash
# Installed by ci/install-hooks.sh — runs the CI matrix locally before pushing.
# Bypass with: git push --no-verify
set -euo pipefail
repo_root=$(git rev-parse --show-toplevel)
exec "$repo_root/ci/run.sh"
HOOK

chmod +x "$hooks_dir/pre-push"
echo "installed: $hooks_dir/pre-push"
echo "bypass with: git push --no-verify"
