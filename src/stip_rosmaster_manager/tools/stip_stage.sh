#!/usr/bin/env bash
set -euo pipefail
case "${1:-}" in
  status) rostopic echo -n 1 /stip/startup/modules ;;
  retry-bringup)
    echo 'IMPORTANT: Confirm robot is stationary and motors safe before retry.'
    read -r -p 'Retry bringup? [y/N] ' a
    [[ "$a" == 'y' || "$a" == 'Y' ]] || exit 1
    rosservice call /stip/startup/retry_bringup
    ;;
  shutdown) rosservice call /stip/startup/shutdown ;;
  restart)
    case "${2:-}" in ui|camera|qr|fire|gas|thermal) ;; *) echo 'Use: restart ui|camera|qr|fire|gas|thermal'; exit 2 ;; esac
    rosservice call "/stip/modules/restart_${2}"
    ;;
  *) echo 'Usage: bash stip_stage.sh {status|retry-bringup|restart ui|camera|qr|fire|gas|thermal|shutdown}'; exit 2 ;;
esac
