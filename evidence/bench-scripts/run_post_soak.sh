#!/usr/bin/env bash
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
while pgrep -x qspy >/dev/null; do sleep 2; done
sleep 2
exec "$S/post_soak.sh"
