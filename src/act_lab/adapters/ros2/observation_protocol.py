"""Bounded real WebSocket checks against the installed Foxglove SDK bridge."""

from __future__ import annotations

import importlib
import json
import struct
import time
from collections import Counter
from typing import Any


def bridge_probe(
    address: str = "ws://127.0.0.1:8765", timeout: float = 15
) -> dict[str, Any]:
    import websocket  # type: ignore[import-not-found]

    messages = importlib.import_module("act_lab_interfaces.msg")
    serialization = importlib.import_module("rclpy.serialization")

    end = time.monotonic() + timeout
    connection = None
    while time.monotonic() < end:
        try:
            connection = websocket.create_connection(
                address, subprotocols=["foxglove.sdk.v1"], timeout=0.5
            )
            break
        except (OSError, websocket.WebSocketException):
            time.sleep(0.05)
    if connection is None:
        raise RuntimeError("bounded Foxglove bridge connection timeout")
    channels: dict[str, Any] = {}
    info = None
    payload = False
    denied: Counter[str] = Counter()
    assets: dict[int, int] = {}
    sent = False
    replies: list[Any] = []
    try:
        while time.monotonic() < end:
            try:
                message = connection.recv()
            except websocket.WebSocketTimeoutException:
                continue
            if isinstance(message, bytes):
                if message and message[0] == 1:
                    telemetry = serialization.deserialize_message(
                        message[13:], messages.ExecutionTelemetry
                    )
                    assert telemetry.schema_version == 1
                    assert len(telemetry.total_effort_nm) == 6
                    payload = True
                elif message and message[0] == 4:
                    request = struct.unpack_from("<I", message, 1)[0]
                    assets[request] = message[5]
                    if request == 1 and message[5] == 0:
                        assert len(message) > 10
            else:
                row = json.loads(message)
                if row.get("op") not in {"advertise", "serverInfo"}:
                    replies.append(row)
                if row.get("op") == "serverInfo":
                    info = row
                    if any(
                        c in row["capabilities"]
                        for c in ("clientPublish", "services", "parameters")
                    ):
                        raise RuntimeError("bridge advertises control capabilities")
                if row.get("op") == "advertise":
                    channels.update({c["topic"]: c for c in row["channels"]})
                    if any(
                        t in channels
                        for t in (
                            "/act_lab/v1/command",
                            "/act_lab/internal/crisp_target",
                        )
                    ):
                        raise RuntimeError("bridge exposes command channel")
                    if "/act_lab/view/telemetry" in channels and not sent:
                        connection.send(
                            json.dumps(
                                dict(
                                    op="subscribe",
                                    subscriptions=[
                                        dict(
                                            id=1,
                                            channelId=channels[
                                                "/act_lab/view/telemetry"
                                            ]["id"],
                                        )
                                    ],
                                )
                            )
                        )
                        attempts = [
                            dict(
                                op="advertise",
                                channels=[
                                    dict(
                                        id=999,
                                        topic="/act_lab/v1/command",
                                        encoding="cdr",
                                        schemaName="act_lab_interfaces/msg/CartesianCommand",
                                    )
                                ],
                            ),
                            dict(
                                op="setParameters",
                                parameters=[
                                    dict(
                                        name="/controller_manager.update_rate", value=1
                                    )
                                ],
                            ),
                            dict(
                                op="getParameters",
                                parameterNames=["/controller_manager.update_rate"],
                                id="probe",
                            ),
                            dict(
                                op="fetchAsset",
                                requestId=1,
                                uri="package://ur_description/meshes/ur5e/visual/base.dae",
                            ),
                            dict(
                                op="fetchAsset", requestId=2, uri="file:///etc/passwd"
                            ),
                        ]
                        for attempt in attempts:
                            connection.send(json.dumps(attempt))
                        # SDK request: opcode, service/call ID, encoding, payload.
                        connection.send_binary(
                            struct.pack("<BIII", 2, 999, 1, 3) + b"cdr"
                        )
                        sent = True
                if (
                    row.get("op") == "serviceCallFailure"
                    and row.get("callId") == 1
                    and row.get("message") == "Server does not support services"
                ):
                    denied["services"] += 1
                if row.get("op") == "status" and row.get("level") == 2:
                    for capability in ("clientPublish", "parameters", "services"):
                        if f"does not support {capability} capability" in row.get(
                            "message", ""
                        ):
                            denied[capability] += 1
            if (
                payload
                and denied["clientPublish"] >= 1
                and denied["parameters"] >= 2
                and denied["services"] >= 1
                and len(assets) == 2
            ):
                break
        if info is None or not payload:
            raise RuntimeError("bridge did not deliver generated telemetry")
        if (
            denied["clientPublish"] < 1
            or denied["parameters"] < 2
            or denied["services"] < 1
        ):
            raise RuntimeError(
                f"bridge control denial unverified: {dict(denied)}; {replies}"
            )
        if assets != {1: 0, 2: 1}:
            raise RuntimeError(f"bridge asset allowlist unverified: {assets}")
        return dict(
            capabilities=info["capabilities"],
            metadata=info.get("metadata"),
            topics=sorted(channels),
            telemetry_delivered=payload,
            denied=dict(denied),
            assets=dict(assets),
        )
    finally:
        connection.close()
