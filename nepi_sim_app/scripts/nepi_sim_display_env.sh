# Resolves DISPLAY and XAUTHORITY for a headless/non-interactive launch, then
# exports them. Meant to be sourced (`. this_file`) at the top of every
# simulator launch_command in ../simulator_launch_targets.yaml.
#
# WHY THIS EXISTS: those launch commands used to hardcode `export DISPLAY=:N`.
# A non-interactive SSH session inherits neither DISPLAY nor XAUTHORITY, so
# SOMETHING has to set them or gzclient and ArduCopter's xterm die instantly
# with "Invalid MIT-MAGIC-COOKIE-1 key" and the launch times out with
# "Timed out waiting for the simulator to become ready" -- with the sim
# half-up, which is a genuinely confusing failure to read.
#
# But the correct display NUMBER is not stable. On this VM it was :1 on
# 2026-08-11 (gdm's greeter owned :0) and :0 after the 2026-08-12 reboot.
# Hardcoding either one means every reboot is a coin flip, and it broke sim
# deployment twice in two days. Detect it instead.
#
# Detection order, most trustworthy first:
#   1. An already-set DISPLAY that actually answers -- respect the caller.
#   2. `who`, which reports the display of each logged-in seat session as a
#      literal ":N" token. This is the real graphical login, which is what we
#      want: it owns the Xauthority cookie we can actually authenticate with.
#   3. Every socket in /tmp/.X11-unix (X0 -> :0, X1 -> :1, ...), as a fallback
#      when nobody is "logged in" by who's reckoning but a server is running.
# Each candidate is PROBED with xdpyinfo (under the resolved XAUTHORITY)
# rather than trusted, since an X socket can exist for a display we cannot
# authenticate against -- that is exactly the :0-owned-by-gdm case that
# started all this. If xdpyinfo is missing we accept the first candidate
# unprobed, which is still strictly better than a hardcoded guess.
#
# Sourcing this never fails the caller: on total detection failure it leaves
# DISPLAY at its best guess and lets the launch proceed, so the failure mode
# stays "gazebo complains" rather than "the script exits before doing
# anything".

# --- XAUTHORITY first: probing a display requires it ---
if [ -z "$XAUTHORITY" ] || [ ! -r "$XAUTHORITY" ]; then
    for _nepi_xauth in \
        "/run/user/$(id -u)/gdm/Xauthority" \
        "$HOME/.Xauthority" \
        "/run/user/$(id -u)/Xauthority"
    do
        if [ -r "$_nepi_xauth" ]; then
            export XAUTHORITY="$_nepi_xauth"
            break
        fi
    done
    unset _nepi_xauth
fi

# --- candidate displays, in priority order, deduplicated ---
_nepi_candidates=""
if [ -n "$DISPLAY" ]; then
    _nepi_candidates="$DISPLAY"
fi
# who's second column is the display for graphical sessions (e.g. ":0")
for _nepi_d in $(who 2>/dev/null | awk '{print $2}' | grep '^:' ); do
    _nepi_candidates="$_nepi_candidates $_nepi_d"
done
# /tmp/.X11-unix/X<N>  ->  :<N>
for _nepi_sock in /tmp/.X11-unix/X* ; do
    [ -e "$_nepi_sock" ] || continue
    _nepi_candidates="$_nepi_candidates :${_nepi_sock##*/X}"
done
unset _nepi_d _nepi_sock

_nepi_display_found=""
_nepi_seen=""
for _nepi_c in $_nepi_candidates; do
    case " $_nepi_seen " in *" $_nepi_c "*) continue ;; esac
    _nepi_seen="$_nepi_seen $_nepi_c"
    if command -v xdpyinfo > /dev/null 2>&1; then
        if DISPLAY="$_nepi_c" xdpyinfo > /dev/null 2>&1; then
            _nepi_display_found="$_nepi_c"
            break
        fi
    else
        # No probe available -- first candidate wins.
        _nepi_display_found="$_nepi_c"
        break
    fi
done

if [ -n "$_nepi_display_found" ]; then
    export DISPLAY="$_nepi_display_found"
elif [ -z "$DISPLAY" ]; then
    # Nothing answered and nothing was set: :0 is the least-surprising guess.
    export DISPLAY=":0"
fi

echo "nepi_sim_display_env: using DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY"
unset _nepi_candidates _nepi_display_found _nepi_seen _nepi_c
