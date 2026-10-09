#!/usr/bin/env bash
# Regenerate the README's static images with the VS Code preview's Mermaid version.
set -euo pipefail

diagram_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
renderer=(npx --yes --package=@mermaid-js/mermaid-cli@11.17.0 --package=mermaid@11.17.0 mmdc)

# For containers requiring browser overrides, supply an existing Puppeteer JSON file.
# Example: MERMAID_PUPPETEER_CONFIG=/path/to/puppeteer.json ./scripts/render-architecture.sh
browser_options=()
if [[ -n "${MERMAID_PUPPETEER_CONFIG:-}" ]]; then
    browser_options=(-p "$MERMAID_PUPPETEER_CONFIG")
fi

for variant in light dark; do
    theme_file="$diagram_root/docs/architecture-$variant.json"
    background="$(node -e 'const fs = require("node:fs"); process.stdout.write(JSON.parse(fs.readFileSync(process.argv[1], "utf8")).themeVariables.background)' "$theme_file")"
    "${renderer[@]}" \
        -i "$diagram_root/docs/architecture.mmd" \
        -o "$diagram_root/docs/architecture-$variant.svg" \
        -c "$theme_file" -b "$background" \
        -I "architecture-$variant" \
        "${browser_options[@]}"
done
