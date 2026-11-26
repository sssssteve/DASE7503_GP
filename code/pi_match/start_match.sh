#!/usr/bin/env bash
# Headless Pi entry. Keep this terminal open for lift confirmations.
set -eo pipefail
root="$(cd -- "$(dirname -- "$0")" && pwd)"
unset DASE_NAV_MONITOR_DIR DASE_DIRECT_NAV_TEST
case "${1:-}" in
  --nav-test)
    shift; action=nav-test
    monitor=0
    while [[ "${1:-}" == --* ]]; do
      case "$1" in
        --monitor) monitor=1 ;;
        --direct) export DASE_DIRECT_NAV_TEST=1 ;;
        *) echo "未知空车测试参数 $1" >&2; exit 2 ;;
      esac
      shift
    done
    if (( monitor )); then
      mkdir -p "$HOME/dase7503_logs"
      DASE_NAV_MONITOR_DIR="$(mktemp -d "$HOME/dase7503_logs/nav-test-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
      export DASE_NAV_MONITOR_DIR
      exec > >(trap '' INT HUP TERM; exec tee "$DASE_NAV_MONITOR_DIR/run.log") 2>&1
      echo "本轮监控日志 $DASE_NAV_MONITOR_DIR"
    fi
    if [[ "${DASE_DIRECT_NAV_TEST:-0}" == 1 ]]; then echo "直达路线空车测试（共用比赛配置）"; fi ;;
  --check) shift; action=config ;;
  --help|-h) echo '用法：bash start_match.sh [--check|--nav-test [--monitor] [--direct]] [实车 match.yaml]'; exit 0 ;;
  --*) echo "未知参数 $1（查看 --help）" >&2; exit 2 ;;
  *) action=headless ;;
esac
if (( $# > 1 )); then echo '只接受一个比赛配置路径' >&2; exit 2; fi
exec bash "$root/pi_hardware/pi.sh" "$action" "${1:-$root/pi_hardware/match.yaml}"
