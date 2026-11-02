#!/bin/bash
#
# Shared ROS 2 environment for this container.
#
# Sourced from two places that MUST behave identically:
#   * /entrypoint.sh -- the container's main process (`docker compose up`)
#   * ~/.bashrc      -- every `docker compose exec ros2 bash`
#
# `docker compose exec` does not run the entrypoint, so anything living only
# in the entrypoint would silently be missing from interactive shells. That
# is why this file exists instead of two copies of the same logic.
#
# Safe to source repeatedly.


# ============================================================
# ROS 2 + workspace
# ============================================================

source /opt/ros/humble/setup.bash

if [ -f /workspace/ROS2-WorkSpace/install/setup.bash ]; then
    source /workspace/ROS2-WorkSpace/install/setup.bash
fi


# ============================================================
# Host network address
#
# The container runs with `network_mode: host`, so it shares the Jetson's
# network namespace outright: whichever network the machine is on -- this
# wifi, a different wifi, an ethernet run, a robot's own access point -- is
# visible in here directly, with no port publishing involved.
#
# Everything below is recomputed LIVE on every call and never cached for
# longer than one command. This machine moves between networks, so an
# address captured when a shell opened is wrong the moment it roams, renews
# a DHCP lease, or gets plugged into ethernet. A stale address is worse than
# none: handed to a robot, it fails silently and looks like a DDS problem.
#
# The discovery server itself needs nothing when the network changes. It
# binds 0.0.0.0, which covers addresses added to the host after it started,
# so only the *reporting* of the address has to keep up.
# ============================================================

# Interfaces that never carry robot traffic: container/VM bridges, veth
# pairs, VPN tunnels, and l4tbr0 -- the Jetson's USB-gadget link, which is
# always 192.168.55.1 and would otherwise look like a perfectly good LAN
# address when no wifi is associated.
_ROS_IGNORED_IFACES='^(lo|docker|br-|veth|virbr|l4tbr|tun|tap)'

# Every address a peer could plausibly reach us on, as "<iface> <address>"
# lines, most-preferred first.
#
# This exists because the default route is not always the robot-facing link:
# wifi for internet and ethernet to the robot at the same time is a normal
# setup, and then the "primary" address is precisely the wrong one to hand
# out. Callers show the alternatives rather than guessing.
ros_host_ips() {
    command ip -o -4 addr show scope global 2>/dev/null \
        | awk -v skip="$_ROS_IGNORED_IFACES" '
            $2 !~ skip {
                split($4, cidr, "/")
                print $2, cidr[1]
            }'
}

# The address a remote peer should use to reach us *right now*.
#
# Prefers the source address of the default route -- that is whichever
# interface is actually carrying traffic, so it tracks a network change with
# nothing hardcoded. Falls back to the first real global address when there
# is no default route at all, which is the normal case on a robot-only
# network or an access point with no internet.
ros_host_ip() {
    local addr

    addr="$(command ip -4 route get 1.1.1.1 2>/dev/null \
            | awk '{for (i = 1; i < NF; i++) if ($i == "src") {print $(i + 1); exit}}')"

    if [ -z "$addr" ]; then
        addr="$(ros_host_ips | awk 'NR == 1 {print $2}')"
    fi

    printf '%s' "$addr"
}

ros_host_iface() {
    local iface

    iface="$(command ip -4 route get 1.1.1.1 2>/dev/null \
             | awk '{for (i = 1; i < NF; i++) if ($i == "dev") {print $(i + 1); exit}}')"

    if [ -z "$iface" ]; then
        iface="$(ros_host_ips | awk 'NR == 1 {print $1}')"
    fi

    printf '%s' "$iface"
}

# Resolves either a literal IPv4 address or a hostname.
#
# On a network you do not control, a hostname is the more durable thing to
# configure: the robot's DHCP address changes, its name usually does not.
# The name has to be resolvable from in here (router DNS, or /etc/hosts) --
# .local/mDNS names need nss-mdns, which this image does not install.
# Prints nothing if the name does not resolve, so callers can drop the entry
# rather than route discovery at an address that does not answer.
ros_resolve_host() {
    local host="$1"

    if printf '%s' "$host" | grep -qE '^[0-9]+(\.[0-9]+){3}$'; then
        printf '%s' "$host"
        return 0
    fi

    getent ahostsv4 "$host" 2>/dev/null | awk 'NR == 1 {print $1}' | tr -d '\n'
}

# Re-reads the current address into the environment.
#
# ROS_HOST_IP is only ever a snapshot -- an exported variable cannot follow
# the network on its own. Call this (or ros_status, which does it for you)
# after switching networks; call ros_host_ip directly when you need a value
# that is live by construction.
ros_refresh_host_ip() {
    ROS_HOST_IP="$(ros_host_ip)"
    ROS_HOST_IFACE="$(ros_host_iface)"
    export ROS_HOST_IP ROS_HOST_IFACE
}

ros_refresh_host_ip

ROS_DISCOVERY_PORT="${ROS_DISCOVERY_PORT:-11811}"
export ROS_DISCOVERY_PORT


# ============================================================
# Fast DDS Discovery Server -- client configuration
#
# Off unless ROS_DISCOVERY_SERVER_ENABLE is explicitly 1 (compose.yaml feeds
# it in from .env, which sets 0). The default is deliberately the OFF side:
# a client with ROS_DISCOVERY_SERVER set routes all of its discovery through
# that server, so defaulting to on would mean an unset variable could leave
# nodes unable to discover each other even locally.
# ============================================================

if [ "${ROS_DISCOVERY_SERVER_ENABLE:-0}" = "1" ]; then

    # Position in this list IS the server id, so the padding semicolons are
    # load-bearing:
    #
    #   id 0 = the server this container starts itself (below).
    #   id 8 = the TurtleBot 4's own server, only if TURTLEBOT4_DISCOVERY_IP
    #          is set. Index 8 is what /etc/turtlebot4_discovery/setup.bash
    #          uses on the robot, and both ends must agree -- the id is what
    #          the server's GUID prefix is derived from.
    #
    # Our own nodes address server 0 as 127.0.0.1 rather than as
    # ${ROS_HOST_IP}, even though the server also listens on the wifi
    # address. Loopback is always up, so this container keeps discovering
    # its OWN nodes across a wifi drop or a DHCP address change. Remote
    # peers reach the exact same server at ${ROS_HOST_IP}:${ROS_DISCOVERY_PORT}
    # -- run `ros_discovery_peer_config` to print what to set on them.
    _ros_discovery_list="127.0.0.1:${ROS_DISCOVERY_PORT}"

    if [ -n "${TURTLEBOT4_DISCOVERY_IP:-}" ]; then
        # Resolved live, so this may be a hostname -- which survives the
        # robot getting a different DHCP address on a different network,
        # where a literal IP would not.
        _ros_tb4="$(ros_resolve_host "${TURTLEBOT4_DISCOVERY_IP}")"

        if [ -n "$_ros_tb4" ]; then
            _ros_discovery_list="${_ros_discovery_list};;;;;;;;${_ros_tb4}:${ROS_DISCOVERY_PORT}"
        else
            # Deliberately dropped rather than passed through unresolved: a
            # server entry that does not answer still gets discovery traffic
            # routed to it, which is worse than having no entry at all.
            echo "ros_env: TURTLEBOT4_DISCOVERY_IP='${TURTLEBOT4_DISCOVERY_IP}'" \
                 "does not resolve -- leaving the robot out of" \
                 "ROS_DISCOVERY_SERVER." >&2
        fi

        unset _ros_tb4
    fi

    export ROS_DISCOVERY_SERVER="${_ros_discovery_list}"
    unset _ros_discovery_list

    # Super client sees every participant the server knows about, not just
    # the ones matching its own subscriptions -- required by `ros2 topic
    # list`, rviz2 and rqt. Not gated on the shell being interactive:
    # launch files and background nodes need the same visibility.
    export ROS_SUPER_CLIENT="${ROS_SUPER_CLIENT:-True}"

else

    # Plain DDS (multicast) discovery. A discovery-server *client* routes ALL
    # discovery through its configured servers, so these have to be absent --
    # not merely empty -- for simple discovery to work at all.
    unset ROS_DISCOVERY_SERVER
    unset ROS_SUPER_CLIENT

fi


# ============================================================
# Fast DDS Discovery Server -- the server process itself
#
# Started by /entrypoint.sh, so `docker compose up` alone leaves a working
# server behind; interactive shells re-check it and restart it if it died.
# ============================================================

ros_discovery_server_listening() {
    ss -uln 2>/dev/null | grep -qE "[:.]${ROS_DISCOVERY_PORT}[[:space:]]"
}

# PID(s) of the running discovery server, found by asking who owns the
# discovery port rather than by matching command lines.
#
# Three traps made the obvious `pkill -f "fastdds discovery"` wrong here:
#
#   * `fastdds discovery` is only a launcher. It spawns the real, long-lived
#     process -- /opt/ros/humble/bin/fast-discovery-server -- and exits,
#     leaving that child reparented to PID 1. Killing the launcher looks
#     like it worked (the python process disappears) while the actual server
#     keeps holding the port.
#
#   * The launcher is python, so its command line reads
#     "python3 .../fastdds.py discovery ...". A pattern of
#     "fastdds discovery" matches neither process.
#
#   * pgrep/pkill -f match against whole command lines, including that of
#     the shell running the check -- AND of every subshell it forks, since a
#     $(...) command substitution inherits the parent's command line. A
#     script that merely mentions the server's name matches itself, and
#     excluding $$ does not save it because the subshell has its own PID.
#     `pkill -f` here kills its own caller: exit 143, server untouched.
#
# The socket owner is immune to all three. The pgrep fallback (for the rare
# case where ss cannot report a PID) uses the [f]irst-character bracket
# trick so that it cannot match a command line quoting the pattern either.
ros_discovery_server_pids() {
    local pids

    pids="$(ss -ulnp 2>/dev/null \
            | awk -v port=":${ROS_DISCOVERY_PORT}\$" '
                $4 ~ port {
                    while (match($0, /pid=[0-9]+/)) {
                        print substr($0, RSTART + 4, RLENGTH - 4)
                        $0 = substr($0, RSTART + RLENGTH)
                    }
                }' \
            | sort -u)"

    if [ -z "$pids" ]; then
        pids="$(pgrep -f '[f]ast-discovery-server' 2>/dev/null)"
    fi

    printf '%s' "$pids"
}

ros_discovery_server_running() {
    ros_discovery_server_listening || [ -n "$(ros_discovery_server_pids)" ]
}

ros_discovery_server_start() {
    if [ "${ROS_DISCOVERY_SERVER_ENABLE:-0}" != "1" ]; then
        return 0
    fi

    # Idempotent: sourced by every interactive shell, so the common case has
    # to be a cheap no-op. The port being bound is the fact that actually
    # matters -- and it also covers a server started outside this container,
    # which shares our network namespace anyway.
    if ros_discovery_server_listening; then
        return 0
    fi

    # No -l flag on purpose. The tool then listens on 0.0.0.0, i.e. on
    # loopback AND the wifi interface at once, so this container's own nodes
    # and remote peers (TurtleBot 4, laptop) all reach the same server.
    # Pinning it to one address would force a choice between the two and
    # would break on every DHCP change.
    nohup fastdds discovery -i 0 -p "${ROS_DISCOVERY_PORT}" \
        > /tmp/discovery_server_0.log 2>&1 &
    disown 2>/dev/null || true

    # Wait for it to claim the port before anything tries to discover
    # through it -- a client that starts first just sits there unmatched.
    local i
    for i in $(seq 1 25); do
        if ros_discovery_server_listening; then
            return 0
        fi
        sleep 0.2
    done

    return 1
}

ros_discovery_server_stop() {
    local pids i

    pids="$(ros_discovery_server_pids)"

    if [ -z "$pids" ]; then
        return 0
    fi

    kill $pids 2>/dev/null || true

    # Wait for the port to actually be released: starting a new server
    # before the old one lets go just makes the new one fail to bind.
    for i in $(seq 1 25); do
        if ! ros_discovery_server_listening; then
            return 0
        fi
        sleep 0.2
    done

    kill -9 $pids 2>/dev/null || true
    return 0
}

ros_discovery_server_restart() {
    ros_discovery_server_stop
    ros_discovery_server_start
}

# Prints the lines to paste on another machine (the TurtleBot 4, a laptop)
# so it discovers through this container's server.
#
# Generated on every call rather than written down anywhere, because the
# address depends on which network this Jetson is on at the time. Re-run it
# after moving networks -- the peer's ROS_DISCOVERY_SERVER has to be updated
# to match, since it names us by address.
ros_discovery_peer_config() {
    local addr others

    addr="$(ros_host_ip)"

    if [ -z "$addr" ]; then
        echo "No usable network address right now -- not associated with any" >&2
        echo "network? Interfaces currently visible:" >&2
        command ip -o -4 addr show 2>/dev/null | awk '{print "  " $2, $4}' >&2
        return 1
    fi

    echo "# On the other machine. This Jetson's address as of right now:"
    echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    echo "export ROS_DISCOVERY_SERVER=\"${addr}:${ROS_DISCOVERY_PORT}\""
    echo "export ROS_SUPER_CLIENT=True"

    others="$(ros_host_ips | awk -v used="$addr" '$2 != used')"

    if [ -n "$others" ]; then
        echo ""
        echo "# We are on more than one network. If the peer is on one of these"
        echo "# instead, use that address in place of ${addr}:"
        printf '%s\n' "$others" | while read -r iface other; do
            echo "#   ${other}  (${iface})"
        done
    fi
}

ros_status() {
    local server_state host_state others

    # Refresh first: this is the command people run after moving networks,
    # so it must not report the address the shell started with.
    ros_refresh_host_ip

    if [ "${ROS_DISCOVERY_SERVER_ENABLE:-0}" != "1" ]; then
        server_state="disabled (plain DDS multicast discovery)"
    elif ros_discovery_server_listening; then
        server_state="listening on 0.0.0.0:${ROS_DISCOVERY_PORT} (server id 0)"
    else
        server_state="NOT RUNNING -- see /tmp/discovery_server_0.log"
    fi

    if [ -n "${ROS_HOST_IP}" ]; then
        host_state="${ROS_HOST_IP} via ${ROS_HOST_IFACE:-?}"
    else
        host_state="none -- no network; remote peers cannot reach us"
    fi

    echo ""
    echo "============================================"
    echo " ROS 2 Humble - Jetson / TurtleBot 4"
    echo "============================================"
    echo "ROS_DISTRO:          ${ROS_DISTRO}"
    echo "ROS_DOMAIN_ID:       ${ROS_DOMAIN_ID}"
    echo "RMW_IMPLEMENTATION:  ${RMW_IMPLEMENTATION}"
    echo "Reachable at:        ${host_state}"

    others="$(ros_host_ips | awk -v used="${ROS_HOST_IP}" '$2 != used')"
    if [ -n "$others" ]; then
        printf '%s\n' "$others" | while read -r iface other; do
            echo "  also on:           ${other} via ${iface}"
        done
    fi

    echo "Discovery server:    ${server_state}"
    echo "ROS_DISCOVERY_SERVER: ${ROS_DISCOVERY_SERVER:-<unset>}"
    echo "ROS_SUPER_CLIENT:    ${ROS_SUPER_CLIENT:-<unset>}"
    echo "Workspace:           /workspace/ROS2-WorkSpace"
    echo "--------------------------------------------"
    echo "ros_status                 this summary (re-reads the address)"
    echo "ros_discovery_peer_config  config for other machines"
    echo "ros_discovery_server_restart"
    echo "============================================"
    echo ""
}
