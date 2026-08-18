#include "tunnel.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <net/if_utun.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/kern_control.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sys_domain.h>
#include <sys/sysctl.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <unistd.h>

#define TUN_PACKET_MAX 1500u

struct tunnel {
    int fd;
    int gateway;
    int configured;
    int forwarding4_changed;
    int forwarding6_changed;
    int pf_enabled;
    char name[IFNAMSIZ];
    char egress[IFNAMSIZ];
    char pf_token[32];
};

static void set_error(char *err, size_t size, const char *message)
{
    if (err && size)
        snprintf(err, size, "%s: %s", message, strerror(errno));
}

static int run_command(const char *const arguments[])
{
    pid_t child = fork();
    int status;
    if (child < 0)
        return -1;
    if (child == 0) {
        execv(arguments[0], (char *const *)arguments);
        _exit(127);
    }
    while (waitpid(child, &status, 0) < 0)
        if (errno != EINTR)
            return -1;
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

static int command_output(const char *const arguments[], char *output,
                          size_t capacity)
{
    int pipes[2], status;
    pid_t child;
    size_t length = 0;

    if (pipe(pipes) != 0)
        return -1;
    child = fork();
    if (child == 0) {
        close(pipes[0]);
        dup2(pipes[1], STDOUT_FILENO);
        dup2(pipes[1], STDERR_FILENO);
        close(pipes[1]);
        execv(arguments[0], (char *const *)arguments);
        _exit(127);
    }
    close(pipes[1]);
    if (child < 0) {
        close(pipes[0]);
        return -1;
    }
    while (length + 1u < capacity) {
        ssize_t got = read(pipes[0], output + length, capacity - length - 1u);
        if (got <= 0)
            break;
        length += (size_t)got;
    }
    output[length] = '\0';
    close(pipes[0]);
    while (waitpid(child, &status, 0) < 0)
        if (errno != EINTR)
            return -1;
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

static int command_input(const char *const arguments[], const char *input)
{
    int pipes[2], status;
    pid_t child;
    size_t length = strlen(input), offset = 0;

    if (pipe(pipes) != 0)
        return -1;
    child = fork();
    if (child == 0) {
        close(pipes[1]);
        dup2(pipes[0], STDIN_FILENO);
        close(pipes[0]);
        execv(arguments[0], (char *const *)arguments);
        _exit(127);
    }
    close(pipes[0]);
    if (child < 0) {
        close(pipes[1]);
        return -1;
    }
    while (offset < length) {
        ssize_t wrote = write(pipes[1], input + offset, length - offset);
        if (wrote < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        offset += (size_t)wrote;
    }
    close(pipes[1]);
    while (waitpid(child, &status, 0) < 0)
        if (errno != EINTR)
            return -1;
    return offset == length && WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

static int default_interface(char out[IFNAMSIZ])
{
    const char *command[] = {"/sbin/route", "-n", "get", "default", NULL};
    char output[2048], *line;

    if (command_output(command, output, sizeof(output)) != 0)
        return -1;
    line = strstr(output, "interface:");
    if (!line || sscanf(line, "interface: %15s", out) != 1) {
        errno = ENODEV;
        return -1;
    }
    return 0;
}

static int enable_forwarding(const char *name, int *changed)
{
    int value = 0, enabled = 1;
    size_t size = sizeof(value);
    if (sysctlbyname(name, &value, &size, NULL, 0) != 0)
        return -1;
    if (value)
        return 0;
    if (sysctlbyname(name, NULL, NULL, &enabled, sizeof(enabled)) != 0)
        return -1;
    *changed = 1;
    return 0;
}

static int configure_pf(tunnel_t *tunnel)
{
    const char *enable[] = {"/sbin/pfctl", "-E", NULL};
    const char *load[] = {"/sbin/pfctl", "-a", "com.apple/audio-modem",
                          "-f", "-", NULL};
    char output[1024], rules[1024], *token;

    if (command_output(enable, output, sizeof(output)) != 0)
        return -1;
    tunnel->pf_enabled = 1;
    token = strstr(output, "Token :");
    if (token)
        (void)sscanf(token, "Token : %31s", tunnel->pf_token);
    snprintf(rules, sizeof(rules),
             "nat on %s inet from 10.77.0.0/30 to any -> (%s)\n"
             "nat on %s inet6 from fd77::/126 to any -> (%s)\n"
             "pass quick on %s inet all keep state\n"
             "pass quick on %s inet6 all keep state\n"
             "pass out quick on %s inet from 10.77.0.0/30 to any keep state\n"
             "pass out quick on %s inet6 from fd77::/126 to any keep state\n",
             tunnel->egress, tunnel->egress, tunnel->egress, tunnel->egress,
             tunnel->name, tunnel->name, tunnel->egress, tunnel->egress);
    return command_input(load, rules);
}

static int configure_tunnel(tunnel_t *tunnel, char *err, size_t err_size)
{
    const char *local = tunnel->gateway ? "10.77.0.1" : "10.77.0.2";
    const char *peer = tunnel->gateway ? "10.77.0.2" : "10.77.0.1";
    const char *ifconfig[] = {"/sbin/ifconfig", tunnel->name, "inet", local,
        peer, "mtu", "1280", "netmask", "255.255.255.255", "up", NULL};
    const char *local6 = tunnel->gateway ? "fd77::1/126" : "fd77::2/126";
    const char *ifconfig6[] = {"/sbin/ifconfig", tunnel->name, "inet6", local6,
        "mtu", "1280", "up", NULL};

    if (run_command(ifconfig) != 0) {
        if (err && err_size)
            snprintf(err, err_size,
                     "cannot assign the IPv4 local/peer addresses to %s",
                     tunnel->name);
        return -1;
    }
    if (run_command(ifconfig6) != 0) {
        if (err && err_size)
            snprintf(err, err_size,
                     "cannot assign the IPv6 local/peer addresses to %s",
                     tunnel->name);
        return -1;
    }
    tunnel->configured = 1;
    if (!tunnel->gateway) {
        const char *route_a[] = {"/sbin/route", "-n", "add", "-net",
            "0.0.0.0/1", "10.77.0.1", NULL};
        const char *route_b[] = {"/sbin/route", "-n", "add", "-net",
            "128.0.0.0/1", "10.77.0.1", NULL};
        const char *route6_a[] = {"/sbin/route", "-n", "add", "-inet6", "-net",
            "::/1", "fd77::1", NULL};
        const char *route6_b[] = {"/sbin/route", "-n", "add", "-inet6", "-net",
            "8000::/1", "fd77::1", NULL};
        if (run_command(route_a) != 0 || run_command(route_b) != 0 ||
            run_command(route6_a) != 0 || run_command(route6_b) != 0) {
            if (err && err_size)
                snprintf(err, err_size, "cannot install client routes on %s",
                         tunnel->name);
            return -1;
        }
        return 0;
    }
    if (default_interface(tunnel->egress) != 0) {
        set_error(err, err_size, "cannot determine gateway internet interface");
        return -1;
    }
    if (enable_forwarding("net.inet.ip.forwarding",
                          &tunnel->forwarding4_changed) != 0 ||
        enable_forwarding("net.inet6.ip6.forwarding",
                          &tunnel->forwarding6_changed) != 0 ||
        configure_pf(tunnel) != 0) {
        errno = EPERM;
        set_error(err, err_size, "cannot enable gateway forwarding/NAT");
        return -1;
    }
    return 0;
}

int tunnel_open(tunnel_t **out, int gateway, int configure,
                char *err, size_t err_size)
{
    tunnel_t *tunnel;
    struct ctl_info info;
    struct sockaddr_ctl address;
    socklen_t name_length;

    if (!out)
        return -1;
    *out = NULL;
    tunnel = calloc(1, sizeof(*tunnel));
    if (!tunnel)
        return -1;
    tunnel->fd = -1;
    tunnel->gateway = gateway;
    tunnel->fd = socket(PF_SYSTEM, SOCK_DGRAM, SYSPROTO_CONTROL);
    if (tunnel->fd < 0) {
        set_error(err, err_size, "cannot create utun socket (run with sudo)");
        tunnel_close(tunnel);
        return -1;
    }
    memset(&info, 0, sizeof(info));
    snprintf(info.ctl_name, sizeof(info.ctl_name), "%s", UTUN_CONTROL_NAME);
    if (ioctl(tunnel->fd, CTLIOCGINFO, &info) != 0) {
        set_error(err, err_size, "cannot find utun kernel control");
        tunnel_close(tunnel);
        return -1;
    }
    memset(&address, 0, sizeof(address));
    address.sc_len = sizeof(address);
    address.sc_family = AF_SYSTEM;
    address.ss_sysaddr = AF_SYS_CONTROL;
    address.sc_id = info.ctl_id;
    address.sc_unit = 0;
    if (connect(tunnel->fd, (struct sockaddr *)&address, sizeof(address)) != 0) {
        set_error(err, err_size, "cannot create utun interface");
        tunnel_close(tunnel);
        return -1;
    }
    name_length = sizeof(tunnel->name);
    if (getsockopt(tunnel->fd, SYSPROTO_CONTROL, UTUN_OPT_IFNAME,
                   tunnel->name, &name_length) != 0) {
        set_error(err, err_size, "cannot read utun interface name");
        tunnel_close(tunnel);
        return -1;
    }
    if (fcntl(tunnel->fd, F_SETFL, fcntl(tunnel->fd, F_GETFL) | O_NONBLOCK) < 0) {
        set_error(err, err_size, "cannot make utun interface nonblocking");
        tunnel_close(tunnel);
        return -1;
    }
    if (configure && configure_tunnel(tunnel, err, err_size) != 0) {
        tunnel_close(tunnel);
        return -1;
    }
    *out = tunnel;
    return 0;
}

int tunnel_fd(const tunnel_t *tunnel)
{
    return tunnel ? tunnel->fd : -1;
}

const char *tunnel_name(const tunnel_t *tunnel)
{
    return tunnel ? tunnel->name : "unknown";
}

ssize_t tunnel_read(tunnel_t *tunnel, uint8_t *packet, size_t capacity)
{
    uint8_t buffer[TUN_PACKET_MAX + sizeof(uint32_t)];
    ssize_t got;
    if (capacity > TUN_PACKET_MAX)
        capacity = TUN_PACKET_MAX;
    got = read(tunnel->fd, buffer, capacity + sizeof(uint32_t));
    if (got <= (ssize_t)sizeof(uint32_t))
        return got < 0 ? got : 0;
    memcpy(packet, buffer + sizeof(uint32_t), (size_t)got - sizeof(uint32_t));
    return got - (ssize_t)sizeof(uint32_t);
}

ssize_t tunnel_write(tunnel_t *tunnel, const uint8_t *packet, size_t length)
{
    uint32_t family = htonl(packet[0] >> 4 == 6 ? AF_INET6 : AF_INET);
    struct iovec vectors[2] = {{&family, sizeof(family)},
                               {(void *)packet, length}};
    ssize_t wrote = writev(tunnel->fd, vectors, 2);
    return wrote < 0 ? wrote : wrote - (ssize_t)sizeof(family);
}

void tunnel_close(tunnel_t *tunnel)
{
    if (!tunnel)
        return;
    if (tunnel->configured && !tunnel->gateway) {
        const char *route_a[] = {"/sbin/route", "-n", "delete", "-net",
            "0.0.0.0/1", "10.77.0.1", NULL};
        const char *route_b[] = {"/sbin/route", "-n", "delete", "-net",
            "128.0.0.0/1", "10.77.0.1", NULL};
        const char *route6_a[] = {"/sbin/route", "-n", "delete", "-inet6",
            "-net", "::/1", "fd77::1", NULL};
        const char *route6_b[] = {"/sbin/route", "-n", "delete", "-inet6",
            "-net", "8000::/1", "fd77::1", NULL};
        (void)run_command(route6_b);
        (void)run_command(route6_a);
        (void)run_command(route_b);
        (void)run_command(route_a);
    }
    if (tunnel->configured && tunnel->gateway) {
        const char *flush[] = {"/sbin/pfctl", "-a", "com.apple/audio-modem",
                               "-F", "all", NULL};
        (void)run_command(flush);
        if (tunnel->pf_enabled && tunnel->pf_token[0]) {
            const char *release[] = {"/sbin/pfctl", "-X", tunnel->pf_token,
                                     NULL};
            (void)run_command(release);
        }
        if (tunnel->forwarding4_changed) {
            int disabled = 0;
            (void)sysctlbyname("net.inet.ip.forwarding", NULL, NULL, &disabled,
                               sizeof(disabled));
        }
        if (tunnel->forwarding6_changed) {
            int disabled = 0;
            (void)sysctlbyname("net.inet6.ip6.forwarding", NULL, NULL, &disabled,
                               sizeof(disabled));
        }
    }
    if (tunnel->fd >= 0)
        close(tunnel->fd);
    free(tunnel);
}
