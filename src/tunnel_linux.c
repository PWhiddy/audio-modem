#include "tunnel.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/if_tun.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>

struct tunnel {
    int fd;
    int gateway;
    int configured;
    int nat_added;
    int forward_out_added;
    int forward_in_added;
    int nat6_added;
    int forward6_out_added;
    int forward6_in_added;
    int old_forwarding;
    int forwarding_changed;
    int old_forwarding6;
    int forwarding6_changed;
    int old_accept_ra;
    int accept_ra_changed;
    char name[IFNAMSIZ];
    char egress[IFNAMSIZ];
};

static void set_error(char *err, size_t size, const char *message)
{
    if (err && size)
        snprintf(err, size, "%s: %s", message, strerror(errno));
}

static int run_command_internal(const char *const arguments[], int quiet)
{
    pid_t child = fork();
    int status;

    if (child < 0)
        return -1;
    if (child == 0) {
        if (quiet) {
            int null_fd = open("/dev/null", O_WRONLY);
            if (null_fd >= 0) {
                (void)dup2(null_fd, STDOUT_FILENO);
                (void)dup2(null_fd, STDERR_FILENO);
                close(null_fd);
            }
        }
        execvp(arguments[0], (char *const *)arguments);
        _exit(127);
    }
    do {
        if (waitpid(child, &status, 0) < 0)
            return -1;
    } while (!WIFEXITED(status) && !WIFSIGNALED(status));
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

static int run_command(const char *const arguments[])
{
    return run_command_internal(arguments, 0);
}

static int run_command_quiet(const char *const arguments[])
{
    return run_command_internal(arguments, 1);
}

static int default_interface(char out[IFNAMSIZ])
{
    FILE *routes = fopen("/proc/net/route", "r");
    char line[512], iface[IFNAMSIZ];
    unsigned destination;

    if (!routes)
        return -1;
    if (!fgets(line, sizeof(line), routes)) {
        fclose(routes);
        errno = EIO;
        return -1;
    }
    while (fgets(line, sizeof(line), routes)) {
        if (sscanf(line, "%15s %x", iface, &destination) == 2 &&
            destination == 0) {
            snprintf(out, IFNAMSIZ, "%s", iface);
            fclose(routes);
            return 0;
        }
    }
    fclose(routes);
    errno = ENODEV;
    return -1;
}

static int forwarding_value(const char *path)
{
    FILE *file = fopen(path, "r");
    int value = 0;
    if (!file)
        return 0;
    if (fscanf(file, "%d", &value) != 1)
        value = 0;
    fclose(file);
    return value != 0;
}

static int set_forwarding(const char *path, int value)
{
    FILE *file = fopen(path, "w");
    if (!file)
        return -1;
    if (fprintf(file, "%d\n", value) < 0) {
        fclose(file);
        return -1;
    }
    return fclose(file);
}

static int add_iptables_rule(const char *const check[],
                             const char *const add[], int *added)
{
    if (run_command_quiet(check) == 0) {
        *added = 1; /* Also clean up a matching rule left by an interrupted run. */
        return 0;
    }
    if (run_command(add) != 0)
        return -1;
    *added = 1;
    return 0;
}

static int configure_tunnel(tunnel_t *tunnel, char *err, size_t err_size)
{
    const char *local = tunnel->gateway ? "10.77.0.1/30" : "10.77.0.2/30";
    const char *address[] = {"ip", "addr", "add", local, "dev",
                             tunnel->name, NULL};
    const char *local6 = tunnel->gateway ? "fd77::1/126" : "fd77::2/126";
    const char *address6[] = {"ip", "-6", "addr", "add", local6, "dev",
                              tunnel->name, NULL};
    const char *link[] = {"ip", "link", "set", "dev", tunnel->name,
                          "mtu", "1280", "up", NULL};

    if (run_command(address) != 0 || run_command(address6) != 0 ||
        run_command(link) != 0) {
        errno = EPERM;
        set_error(err, err_size, "cannot configure TUN interface");
        return -1;
    }
    tunnel->configured = 1;
    if (!tunnel->gateway) {
        const char *route_a[] = {"ip", "route", "add", "0.0.0.0/1", "via",
                                 "10.77.0.1", "dev", tunnel->name, NULL};
        const char *route_b[] = {"ip", "route", "add", "128.0.0.0/1", "via",
                                 "10.77.0.1", "dev", tunnel->name, NULL};
        const char *route6_a[] = {"ip", "-6", "route", "add", "::/1", "via",
                                  "fd77::1", "dev", tunnel->name, NULL};
        const char *route6_b[] = {"ip", "-6", "route", "add", "8000::/1",
                                  "via", "fd77::1", "dev", tunnel->name, NULL};
        if (run_command(route_a) != 0 || run_command(route_b) != 0 ||
            run_command(route6_a) != 0 || run_command(route6_b) != 0) {
            errno = EPERM;
            set_error(err, err_size, "cannot install client routes");
            return -1;
        }
        return 0;
    }

    if (default_interface(tunnel->egress) != 0) {
        set_error(err, err_size, "cannot determine gateway internet interface");
        return -1;
    }
    {
        char accept_ra_path[256];
        snprintf(accept_ra_path, sizeof(accept_ra_path),
                 "/proc/sys/net/ipv6/conf/%s/accept_ra", tunnel->egress);
        tunnel->old_accept_ra = forwarding_value(accept_ra_path);
        if (tunnel->old_accept_ra != 2 &&
            set_forwarding(accept_ra_path, 2) == 0)
            tunnel->accept_ra_changed = 1;
    }
    tunnel->old_forwarding = forwarding_value("/proc/sys/net/ipv4/ip_forward");
    if (!tunnel->old_forwarding &&
        set_forwarding("/proc/sys/net/ipv4/ip_forward", 1) != 0) {
        set_error(err, err_size, "cannot enable IPv4 forwarding");
        return -1;
    }
    if (!tunnel->old_forwarding)
        tunnel->forwarding_changed = 1;
    tunnel->old_forwarding6 =
        forwarding_value("/proc/sys/net/ipv6/conf/all/forwarding");
    if (!tunnel->old_forwarding6 &&
        set_forwarding("/proc/sys/net/ipv6/conf/all/forwarding", 1) != 0) {
        set_error(err, err_size, "cannot enable IPv6 forwarding");
        return -1;
    }
    if (!tunnel->old_forwarding6)
        tunnel->forwarding6_changed = 1;
    {
        const char *nat_check[] = {"iptables", "-w", "-t", "nat", "-C",
            "POSTROUTING", "-s", "10.77.0.0/30", "-o", tunnel->egress,
            "-j", "MASQUERADE", NULL};
        const char *nat_add[] = {"iptables", "-w", "-t", "nat", "-A",
            "POSTROUTING", "-s", "10.77.0.0/30", "-o", tunnel->egress,
            "-j", "MASQUERADE", NULL};
        const char *out_check[] = {"iptables", "-w", "-C", "FORWARD", "-i",
            tunnel->name, "-o", tunnel->egress, "-j", "ACCEPT", NULL};
        const char *out_add[] = {"iptables", "-w", "-A", "FORWARD", "-i",
            tunnel->name, "-o", tunnel->egress, "-j", "ACCEPT", NULL};
        const char *in_check[] = {"iptables", "-w", "-C", "FORWARD", "-i",
            tunnel->egress, "-o", tunnel->name, "-m", "conntrack",
            "--ctstate", "ESTABLISHED,RELATED", "-j", "ACCEPT", NULL};
        const char *in_add[] = {"iptables", "-w", "-A", "FORWARD", "-i",
            tunnel->egress, "-o", tunnel->name, "-m", "conntrack",
            "--ctstate", "ESTABLISHED,RELATED", "-j", "ACCEPT", NULL};
        if (add_iptables_rule(nat_check, nat_add, &tunnel->nat_added) != 0 ||
            add_iptables_rule(out_check, out_add,
                              &tunnel->forward_out_added) != 0 ||
            add_iptables_rule(in_check, in_add,
                              &tunnel->forward_in_added) != 0) {
            errno = EPERM;
            set_error(err, err_size, "cannot install gateway firewall rules");
            return -1;
        }
    }
    {
        const char *nat_check[] = {"ip6tables", "-w", "-t", "nat", "-C",
            "POSTROUTING", "-s", "fd77::/126", "-o", tunnel->egress,
            "-j", "MASQUERADE", NULL};
        const char *nat_add[] = {"ip6tables", "-w", "-t", "nat", "-A",
            "POSTROUTING", "-s", "fd77::/126", "-o", tunnel->egress,
            "-j", "MASQUERADE", NULL};
        const char *out_check[] = {"ip6tables", "-w", "-C", "FORWARD", "-i",
            tunnel->name, "-o", tunnel->egress, "-j", "ACCEPT", NULL};
        const char *out_add[] = {"ip6tables", "-w", "-A", "FORWARD", "-i",
            tunnel->name, "-o", tunnel->egress, "-j", "ACCEPT", NULL};
        const char *in_check[] = {"ip6tables", "-w", "-C", "FORWARD", "-i",
            tunnel->egress, "-o", tunnel->name, "-m", "conntrack",
            "--ctstate", "ESTABLISHED,RELATED", "-j", "ACCEPT", NULL};
        const char *in_add[] = {"ip6tables", "-w", "-A", "FORWARD", "-i",
            tunnel->egress, "-o", tunnel->name, "-m", "conntrack",
            "--ctstate", "ESTABLISHED,RELATED", "-j", "ACCEPT", NULL};
        if (add_iptables_rule(nat_check, nat_add, &tunnel->nat6_added) != 0 ||
            add_iptables_rule(out_check, out_add,
                              &tunnel->forward6_out_added) != 0 ||
            add_iptables_rule(in_check, in_add,
                              &tunnel->forward6_in_added) != 0) {
            errno = EPERM;
            set_error(err, err_size, "cannot install IPv6 gateway firewall rules");
            return -1;
        }
    }
    return 0;
}

int tunnel_open(tunnel_t **out, int gateway, int configure,
                char *err, size_t err_size)
{
    tunnel_t *tunnel;
    struct ifreq request;

    if (!out)
        return -1;
    *out = NULL;
    tunnel = calloc(1, sizeof(*tunnel));
    if (!tunnel)
        return -1;
    tunnel->fd = -1;
    tunnel->gateway = gateway;
    tunnel->fd = open("/dev/net/tun", O_RDWR | O_CLOEXEC);
    if (tunnel->fd < 0) {
        set_error(err, err_size, "cannot open /dev/net/tun (run with sudo)");
        tunnel_close(tunnel);
        return -1;
    }
    memset(&request, 0, sizeof(request));
    request.ifr_flags = IFF_TUN | IFF_NO_PI;
    snprintf(request.ifr_name, sizeof(request.ifr_name), "amodem%%d");
    if (ioctl(tunnel->fd, TUNSETIFF, &request) < 0) {
        set_error(err, err_size, "cannot create TUN interface (run with sudo)");
        tunnel_close(tunnel);
        return -1;
    }
    snprintf(tunnel->name, sizeof(tunnel->name), "%s", request.ifr_name);
    if (fcntl(tunnel->fd, F_SETFL, fcntl(tunnel->fd, F_GETFL) | O_NONBLOCK) < 0) {
        set_error(err, err_size, "cannot make TUN interface nonblocking");
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
    return read(tunnel->fd, packet, capacity);
}

ssize_t tunnel_write(tunnel_t *tunnel, const uint8_t *packet, size_t length)
{
    return write(tunnel->fd, packet, length);
}

void tunnel_close(tunnel_t *tunnel)
{
    if (!tunnel)
        return;
    if (tunnel->configured && !tunnel->gateway) {
        const char *route_a[] = {"ip", "route", "del", "0.0.0.0/1", "via",
                                 "10.77.0.1", "dev", tunnel->name, NULL};
        const char *route_b[] = {"ip", "route", "del", "128.0.0.0/1", "via",
                                 "10.77.0.1", "dev", tunnel->name, NULL};
        const char *route6_a[] = {"ip", "-6", "route", "del", "::/1", "via",
                                  "fd77::1", "dev", tunnel->name, NULL};
        const char *route6_b[] = {"ip", "-6", "route", "del", "8000::/1",
                                  "via", "fd77::1", "dev", tunnel->name, NULL};
        (void)run_command(route6_b);
        (void)run_command(route6_a);
        (void)run_command(route_b);
        (void)run_command(route_a);
    }
    if (tunnel->configured && tunnel->gateway) {
        if (tunnel->forward6_in_added) {
            const char *rule[] = {"ip6tables", "-w", "-D", "FORWARD", "-i",
                tunnel->egress, "-o", tunnel->name, "-m", "conntrack",
                "--ctstate", "ESTABLISHED,RELATED", "-j", "ACCEPT", NULL};
            (void)run_command(rule);
        }
        if (tunnel->forward6_out_added) {
            const char *rule[] = {"ip6tables", "-w", "-D", "FORWARD", "-i",
                tunnel->name, "-o", tunnel->egress, "-j", "ACCEPT", NULL};
            (void)run_command(rule);
        }
        if (tunnel->nat6_added) {
            const char *rule[] = {"ip6tables", "-w", "-t", "nat", "-D",
                "POSTROUTING", "-s", "fd77::/126", "-o", tunnel->egress,
                "-j", "MASQUERADE", NULL};
            (void)run_command(rule);
        }
        if (tunnel->forward_in_added) {
            const char *rule[] = {"iptables", "-w", "-D", "FORWARD", "-i",
                tunnel->egress, "-o", tunnel->name, "-m", "conntrack",
                "--ctstate", "ESTABLISHED,RELATED", "-j", "ACCEPT", NULL};
            (void)run_command(rule);
        }
        if (tunnel->forward_out_added) {
            const char *rule[] = {"iptables", "-w", "-D", "FORWARD", "-i",
                tunnel->name, "-o", tunnel->egress, "-j", "ACCEPT", NULL};
            (void)run_command(rule);
        }
        if (tunnel->nat_added) {
            const char *rule[] = {"iptables", "-w", "-t", "nat", "-D",
                "POSTROUTING", "-s", "10.77.0.0/30", "-o", tunnel->egress,
                "-j", "MASQUERADE", NULL};
            (void)run_command(rule);
        }
        if (tunnel->forwarding_changed)
            (void)set_forwarding("/proc/sys/net/ipv4/ip_forward", 0);
        if (tunnel->forwarding6_changed)
            (void)set_forwarding("/proc/sys/net/ipv6/conf/all/forwarding", 0);
        if (tunnel->accept_ra_changed) {
            char accept_ra_path[256];
            snprintf(accept_ra_path, sizeof(accept_ra_path),
                     "/proc/sys/net/ipv6/conf/%s/accept_ra", tunnel->egress);
            (void)set_forwarding(accept_ra_path, tunnel->old_accept_ra);
        }
    }
    if (tunnel->fd >= 0)
        close(tunnel->fd);
    free(tunnel);
}
