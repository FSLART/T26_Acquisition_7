#!/usr/bin/env bash
# Picks the next firmware version and writes it to VERSION. Run by CI on every push to main,
# from the repository folder, with the tags fetched (actions/checkout fetch-depth: 0).
#
#   - no v* tag yet                       -> keep VERSION as it is (first release)
#   - VERSION is higher than the last tag -> keep it (MINOR/MAJOR bumped by hand in the PR)
#   - otherwise                           -> last tag's PATCHLEVEL + 1, MINOR/MAJOR from the tag
#
# Prints the chosen version (X.Y.Z) on stdout. Commits and tags are left to the workflow.
set -euo pipefail

file=${1:-VERSION}

field() { sed -n "s/^$1 *= *\([0-9]*\).*/\1/p" "$file"; }

major=$(field VERSION_MAJOR)
minor=$(field VERSION_MINOR)
patch=$(field PATCHLEVEL)
if [[ -z "$major" || -z "$minor" || -z "$patch" ]]; then
    echo "error: $file has no VERSION_MAJOR / VERSION_MINOR / PATCHLEVEL" >&2
    exit 1
fi

# Highest vX.Y.Z tag (version order, not date)
last=$(git tag --list 'v[0-9]*.[0-9]*.[0-9]*' | sed 's/^v//' | sort -t. -k1,1n -k2,2n -k3,3n | tail -n1)

if [[ -n "$last" ]]; then
    IFS=. read -r tmajor tminor tpatch <<<"$last"
    # Compare as numbers: is VERSION > last tag?
    if (( major > tmajor || (major == tmajor && (minor > tminor || (minor == tminor && patch > tpatch))) )); then
        echo "VERSION $major.$minor.$patch is above the last tag v$last: kept" >&2
    else
        major=$tmajor
        minor=$tminor
        patch=$((tpatch + 1))
        echo "last tag v$last: patch bump" >&2
    fi
else
    echo "no v* tag yet: VERSION kept" >&2
fi

sed -i -e "s/^VERSION_MAJOR *=.*/VERSION_MAJOR = $major/" \
       -e "s/^VERSION_MINOR *=.*/VERSION_MINOR = $minor/" \
       -e "s/^PATCHLEVEL *=.*/PATCHLEVEL = $patch/" "$file"

echo "$major.$minor.$patch"
