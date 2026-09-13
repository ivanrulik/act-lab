# ADR 007: Held-key X11 keyboard control

- Status: accepted
- Date: 2026-09-08
- Supersedes: ADR 005 keyboard input decision

## Context

MuJoCo's passive-viewer callback reports key presses but not releases, so the
tap-to-jog design in ADR 005 cannot provide fluid velocity control or detect a
released deadman. The viewer also assigns rendering actions to ordinary keys,
making Space unsuitable as the deadman while operating the robot.

## Decision

Keyboard teleoperation uses an optional X11 adapter built from `pynput` and
`python-xlib`. A background listener records press and release state, and a
background focus monitor requires the MuJoCo viewer to own input focus. Neither
thread accesses the simulator; the 50 Hz control thread consumes a locked key
snapshot and emits the domain `Action`.

Either Shift key is the held deadman. `W/S`, `A/D`, and `R/F` command continuous
world-frame translation with normalized diagonals, while `O/C` adjust a latched
gripper target. Releasing Shift, releasing all command keys, or losing focus
emits a disabled measured-pose hold on the next control cycle. `Q` is handled by
both the X11 listener and MuJoCo's press callback so it reliably closes the
viewer and process.

Shutdown joins the input workers and closes the focus display. With pinned
`pynput` 1.8, XRecord disable is sent on the listener's separate stop connection
and flushed before joining; the recording connection is blocked awaiting replies.
With pinned
MuJoCo 3.12, `Handle.close()` only requests exit and `is_running()` immediately
becomes false. The keyboard adapter therefore waits for the handle's internal
simulation weak reference to expire after render destruction (with a five-second
deadline), before allowing interpreter/GLFW cleanup. This version-specific seam
must be checked on MuJoCo upgrades; it avoids racing GLFW termination against
the still-running render thread.

The X11 dependencies are confined to the `ui` optional dependency group and
Docker target. Headless, training, evaluation, and noninteractive simulation
profiles do not install or import them.

## Consequences

Keyboard motion is continuous and release-aware while retaining the existing
application safety boundary. Operators must run an X11 session and keep the
viewer focused. Wayland-only environments require XWayland or a future native
input adapter. The gripper target remains latched across contact deflection so
arm motion does not unintentionally release an object.
