import argparse
import json
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="robot-scratch: fake/replay/discovery and explicit live teleop"
    )
    sub = parser.add_subparsers(dest="command", required=True)
    validate = sub.add_parser("validate", help="static graph preflight; constructs no devices")
    validate.add_argument("config")
    validate.add_argument(
        "--dry-run", action="store_true", help="print the validated fake execution plan"
    )
    run = sub.add_parser(
        "run", help="run deterministic fake teleop, stub policy and takeover scenario"
    )
    run.add_argument("config")
    run.add_argument("--output", required=True)
    run.add_argument("--steps", type=int, default=12)
    read = sub.add_parser(
        "replay", help="validate and read event timeline; never executes commands"
    )
    read.add_argument("recording")
    read.add_argument(
        "--jsonl", action="store_true", help="print validated events instead of summary"
    )
    collect_check = sub.add_parser(
        "collect-validate", help="validate independent multi-endpoint FAKE plan; no devices"
    )
    collect_check.add_argument("config")
    collect_check.add_argument("--dry-run", action="store_true")
    collect_run = sub.add_parser(
        "collect-demo", help="record fake multi-endpoint/camera episode with source replacement"
    )
    collect_run.add_argument("config")
    collect_run.add_argument("--output", required=True)
    collect_run.add_argument("--steps", type=int, default=20)
    collect_read = sub.add_parser(
        "collect-read", help="stream-validate fake collection MCAP; never creates devices"
    )
    collect_read.add_argument("recording")
    from .discovery import add_arguments, execute

    add_arguments(
        sub.add_parser("discover", help="passive OS metadata and explicit identity-role matching")
    )
    from .session_launcher import add_arguments as add_teleop_arguments

    add_teleop_arguments(
        sub.add_parser(
            "teleop", help="start explicit device-host teleop session; connects hardware"
        )
    )
    from .deployment import add_arguments as add_device_arguments

    add_device_arguments(
        sub.add_parser(
            "collect", help="device-local real episode collection with explicit task/numbering"
        ),
        True,
    )
    add_device_arguments(
        sub.add_parser("device-teleop", help="device-local free teleop without dataset recording"),
        False,
    )
    setup = sub.add_parser(
        "collect-configure", help="write a separate collection profile set; no devices or motion"
    )
    setup.add_argument("--deployment", required=True)
    setup.add_argument("--output", required=True)
    setup.add_argument("--trigger-pressed", required=True, choices=["open", "close"])
    setup.add_argument("--teleop-start", required=True, choices=["preserve", "follow_trigger"])
    setup.add_argument("--image-storage", required=True, choices=["raw_mcap", "mp4"])
    setup.add_argument("--depth-role", action="append", default=[])
    setup.add_argument(
        "--depth-profile", nargs=3, type=int, metavar=("WIDTH", "HEIGHT", "FPS"), required=True
    )
    qc = sub.add_parser("episode-qc", help="offline real MCAP integrity/QC; never connects devices")
    qc.add_argument("path")
    qc.add_argument("--deep", action="store_true")
    audit = sub.add_parser(
        "episode-audit", help="offline timing/integrity report outside source episode; no devices"
    )
    audit.add_argument("path")
    audit.add_argument("--output", required=True)
    audit.add_argument("--deep", action="store_true")
    view = sub.add_parser(
        "training-view",
        help="causal state/command/image-reference audit view; no LeRobot or devices",
    )
    view.add_argument("path")
    view.add_argument("--config", required=True)
    view.add_argument("--output", required=True)
    view.add_argument(
        "--journal",
        action="append",
        default=[],
        help="optional pre-episode target evidence; repeat in chronological segment order",
    )
    graph = sub.add_parser(
        "experiment-plan",
        help="compile typed collection/replay/inference plan; never launches devices",
    )
    graph.add_argument("config")
    trajectory = sub.add_parser(
        "trajectory-check", help="build offline confirmed-target timeline; never executes hardware"
    )
    trajectory.add_argument("path")
    trajectory.add_argument("--config", required=True)
    trajectory.add_argument("--output", required=True)
    workflow = sub.add_parser(
        "workflow-run",
        help="native trajectory/mock-policy session; physical execution requires --execute",
    )
    workflow.add_argument("config")
    workflow.add_argument("--output", required=True)
    workflow.add_argument("--deployment")
    workflow.add_argument("--execute", action="store_true")
    workflow.add_argument("--check-only", action="store_true")
    args = parser.parse_args(argv)
    if args.command == "collect-configure":
        try:
            from .deployment import prepare_collection

            result = prepare_collection(
                args.deployment,
                args.output,
                trigger_pressed=args.trigger_pressed,
                teleop_start=args.teleop_start,
                image_storage=args.image_storage,
                depth_roles=args.depth_role,
                depth_profile=args.depth_profile,
            )
            print(json.dumps(result))
            return 0
        except (ValueError, KeyError, OSError) as exc:
            print(json.dumps({"error": str(exc)}), file=sys.stderr)
            return 2
    if args.command == "workflow-run":
        try:
            from .workflow_runner import run_workflow

            result = run_workflow(
                args.config,
                args.output,
                deployment=args.deployment,
                execute=args.execute,
                check_only=args.check_only,
            )
            print(json.dumps(result))
            return 0 if result.get("state") in (None, "COMPLETED", "STOPPED") else 3
        except (ValueError, KeyError, OSError, RuntimeError) as exc:
            print(json.dumps({"error": str(exc)}), file=sys.stderr)
            return 2
    if args.command in ("experiment-plan", "trajectory-check"):
        try:
            from pathlib import Path

            if args.command == "experiment-plan":
                from .experiment_graph import compile_experiment

                result = compile_experiment(
                    json.loads(Path(args.config).read_text(encoding="utf-8-sig"))
                )
            else:
                from .episode_store import write_json
                from .trajectory import trajectory_plan

                result = trajectory_plan(
                    args.path, json.loads(Path(args.config).read_text(encoding="utf-8-sig"))
                )
                output = Path(args.output).resolve()
                source = Path(args.path).resolve()
                if output == source or output.is_relative_to(source):
                    raise ValueError("trajectory output must be outside episode")
                output.parent.mkdir(parents=True, exist_ok=True)
                write_json(output, result)
                result = {k: v for k, v in result.items() if k != "events"}
            print(json.dumps(result))
            return 0
        except (ValueError, KeyError, OSError, RuntimeError) as exc:
            print(json.dumps({"error": str(exc)}), file=sys.stderr)
            return 2
    if args.command == "training-view":
        try:
            from pathlib import Path

            from .training_view import build_training_view

            result = build_training_view(
                args.path,
                json.loads(Path(args.config).read_text(encoding="utf-8-sig")),
                args.output,
                journal_paths=args.journal,
            )
            print(json.dumps(result))
            return 0
        except (ValueError, KeyError, OSError, RuntimeError) as exc:
            print(json.dumps({"error": str(exc)}), file=sys.stderr)
            return 2
    if args.command == "episode-audit":
        try:
            from .data_audit import audit_episode

            report = audit_episode(args.path, args.output, deep=args.deep)
            print(
                json.dumps(
                    {
                        "episode": report["episode"],
                        "qc_passed": report["qc"]["passed"],
                        "warnings": report["warnings"],
                        "output": args.output,
                    }
                )
            )
            return 0
        except (ValueError, KeyError, OSError, RuntimeError) as exc:
            print(json.dumps({"error": str(exc)}), file=sys.stderr)
            return 2
    if args.command in ("collect", "device-teleop", "episode-qc"):
        try:
            if args.command == "episode-qc":
                from pathlib import Path

                from .episode_qc import quality_report

                root = Path(args.path)
                manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
                if manifest.get("schema_version") not in ("real-episode/1", "real-episode/2"):
                    raise ValueError("unsupported real episode manifest")
                result = quality_report(root, manifest, deep=args.deep)
                print(json.dumps(result, ensure_ascii=True))
                return 0 if result["passed"] else 3
            from .deployment import launch

            launch(args)
            return 0
        except (ValueError, KeyError, OSError, RuntimeError) as exc:
            print(json.dumps({"error": str(exc)}, ensure_ascii=True), file=sys.stderr)
            return 2
    if args.command == "teleop":
        from .session_launcher import run as launch

        try:
            launch(args)
            return 0
        except (ValueError, OSError, RuntimeError) as exc:
            print(json.dumps({"error": str(exc)}), file=sys.stderr)
            return 2
    if args.command.startswith("collect-"):
        try:
            from pathlib import Path

            from .collection import validate_collection
            from .graph import strict_json

            if args.command == "collect-read":
                from .collection_episode import read_collection

                result = read_collection(args.recording)
            else:
                plan = validate_collection(
                    strict_json(Path(args.config).read_text(encoding="utf-8"))
                )
                if args.command == "collect-demo":
                    from .collection_demo import run_collection_demo

                    result = run_collection_demo(plan["config"], args.output, steps=args.steps)
                else:
                    result = {
                        "valid": True,
                        "mode": "fake",
                        "physical_commands": False,
                        "config_hash": plan["config_hash"],
                        "batches": plan["batches"],
                    }
                    if args.dry_run:
                        result["plan"] = plan
            print(json.dumps(result, sort_keys=True, allow_nan=False))
            return 0
        except (ValueError, OSError, PermissionError) as exc:
            print(json.dumps({"error": str(exc)}, ensure_ascii=False), file=sys.stderr)
            return 2
    if args.command == "discover":
        try:
            result, code = execute(args)
            print(json.dumps(result, sort_keys=True, allow_nan=False))
            return code
        except (ValueError, OSError) as exc:
            print(json.dumps({"error": str(exc)}, ensure_ascii=False), file=sys.stderr)
            return 2
    # Passive discovery is usable before installing runtime dependencies.
    from .graph import ConfigError, load_graph
    from .recording import RecordingError, replay, summary

    try:
        if args.command == "validate":
            plan = load_graph(args.config)
            result = {"mode": "fake", "valid": True, "config_hash": plan["config_hash"]}
            if args.dry_run:
                result["plan"] = plan
        elif args.command == "run":
            from .app import run_demo

            result = run_demo(load_graph(args.config)["config"], args.output, steps=args.steps)
        else:
            events = replay(args.recording)
            if args.jsonl:
                for event in events:
                    print(json.dumps(event, sort_keys=True, allow_nan=False))
                return 0
            result = summary(events)
        print(json.dumps(result, sort_keys=True, allow_nan=False))
        return 0
    except (ConfigError, RecordingError, ValueError, OSError, PermissionError) as exc:
        print(json.dumps({"error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
