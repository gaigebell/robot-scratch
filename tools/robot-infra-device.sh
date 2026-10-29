#!/usr/bin/env bash
# Install under <device-root>/bin/. No Windows, proxy, CDN or SSH dependency.
set -eo pipefail
device_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
deployment="${ROBOT_INFRA_DEPLOYMENT:-$device_root/deployment.json}"
# Select the same manifest for environment setup and the Python CLI.
# Keep the last explicit occurrence, matching argparse's normal behavior.
explicit_deployment=false
arguments=("$@")
for ((index=0; index<${#arguments[@]}; index++)); do
  case "${arguments[index]}" in
    --deployment)
      if ((index+1>=${#arguments[@]})) || [[ -z "${arguments[index+1]}" || "${arguments[index+1]}" == --* ]]; then
        echo "--deployment requires a file path" >&2; exit 2
      fi
      deployment="${arguments[index+1]}"; explicit_deployment=true
      index=$((index+1)) ;;
    --deployment=*) deployment="${arguments[index]#--deployment=}"; explicit_deployment=true ;;
  esac
done
test -n "$deployment" || { echo "--deployment requires a file path" >&2; exit 2; }
deployment="$(python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).expanduser().resolve())' "$deployment")"
setup_output="$(python3 -c 'import json,sys,pathlib; p=pathlib.Path(sys.argv[1]); c=json.loads(p.read_text(encoding="utf-8-sig")); print("\n".join(str((p.parent/pathlib.Path(x).expanduser()).resolve()) for x in c["setup_scripts"]))' "$deployment")"
setup_scripts=()
if [[ -n "$setup_output" ]]; then mapfile -t setup_scripts <<< "$setup_output"; fi
for setup_script in "${setup_scripts[@]}"; do
  test -f "$setup_script" || { echo "Missing ROS setup: $setup_script" >&2; exit 2; }
  source "$setup_script"
done
python_paths="$(python3 -c 'import json,sys,pathlib,os; p=pathlib.Path(sys.argv[1]); c=json.loads(p.read_text(encoding="utf-8-sig")); print(os.pathsep.join(str((p.parent/pathlib.Path(x).expanduser()).resolve()) for x in c["python_paths"]))' "$deployment")"
export PYTHONPATH="$python_paths${PYTHONPATH:+:$PYTHONPATH}"
case "${1:-}" in
  collect|device-teleop)
    if "$explicit_deployment"; then exec python3 -m robot_infra "$@"
    else exec python3 -m robot_infra "$@" --deployment "$deployment"; fi ;;
  *) exec python3 -m robot_infra "$@" ;;
esac
