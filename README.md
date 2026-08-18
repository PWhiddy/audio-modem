# audio-modem

`audio-modem` is a small userspace internet gateway whose only data link is
sound. One machine runs as a gateway and shares its internet connection; a
second machine creates a TUN/utun interface and transparently routes normal IP
traffic through its microphone and speaker.

The implementation is written in C and supports Linux and macOS. It uses the
native audio stack on each platform, so the build has no downloaded or vendored
dependencies and does not use CMake.

## Build

```sh
git clone <repository-url>
cd audio-modem
make
make test
```

Linux needs a C11 compiler, `make`, and the standard ALSA runtime library
(`libasound.so.2`). Running the automatic network setup also needs `ip`,
`iptables`, and `ip6tables`. No ALSA development package is required. macOS
needs the Xcode command-line tools; CoreAudio and the networking frameworks are
part of the operating system.

## Run

Place each machine's speaker near the other machine's microphone, or use a
suitable analog audio connection. Start the internet-connected machine first:

```sh
sudo ./audio-modem --gateway
```

Then start the client:

```sh
sudo ./audio-modem --client
```

`--client` is the default, so `sudo ./audio-modem` is equivalent. Elevated
privileges are needed to create and configure TUN/utun devices, routes,
forwarding, and NAT. On macOS, allow microphone access for the terminal when
prompted. Stop with Ctrl-C so the program can remove the routes and forwarding
rules it installed.

Once the console reports `link up`, applications use the tunnel without proxy
settings. IPv4 and IPv6 routes are installed on the client. The gateway uses
the existing default internet interface for forwarding and NAT. An upstream
connection without IPv6 will naturally only provide working IPv4 service.

The program logs its selected input and output devices, tunnel configuration,
connection handshake, negotiated audio band, link transitions, retries, and
periodic packet counters. Routine idle polls are intentionally not logged.

The acoustic path is bidirectional: the client speaker must reach the gateway
microphone, and the gateway speaker must reach the client microphone. While a
side is waiting for the handshake, it reports its input RMS/peak level and best
modem-sync score every five seconds. A score near zero means the selected
microphone is not hearing a recognizable burst. A score approaching `0.55`
means the burst is recognizable; `candidate frame rejected` means it found the
OFDM preamble but the frame was too damaged to pass error correction and CRC.
Move the relevant microphone closer or adjust output/input gain, while avoiding
peaks near 0 dBFS because those indicate clipping.

## Options

```text
--client              run as the client (default)
--gateway             run as the internet gateway
--band LOW:HIGH       requested band in Hz (default 2000:12000)
--input DEVICE        ALSA device name or CoreAudio device UID
--output DEVICE       ALSA device name or CoreAudio device UID
--no-config           do not install addresses, routes, forwarding, or NAT
--self-test           test the codec without audio hardware or root
--help                show command help
```

On Linux, `arecord -L` and `aplay -L` list usable ALSA names. The startup log
resolves the selected PCM to its backing card/device when ALSA exposes that
information. PipeWire or PulseAudio may intentionally expose only a virtual
default; `wpctl status` or `pactl list short sources` / `sinks` then shows which
physical endpoints that sound server currently routes to. Use `--input pulse
--output pulse` to select the current PulseAudio defaults explicitly. When run
through `sudo`, audio-modem connects to the invoking desktop user's sound server
while retaining the privileges needed for TUN and network setup; no PulseAudio
environment variables need to be passed through `sudo`. The defaults are
normally best on both platforms. The client and gateway may request different
bands; the handshake selects their overlap. Initial discovery always uses the
full 2-12 kHz bootstrap band.

`--no-config` is intended for development and custom network setups. It still
creates the TUN/utun device and therefore normally still requires elevated
privileges.

## How it works

- 48 kHz mono audio, with active OFDM carriers inside the negotiated 2-12 kHz
  range.
- QPSK (4-QAM) data carriers, BPSK pilots and training symbols, cyclic prefixes,
  and per-burst channel/phase estimation. Pilot tracking compensates for small
  sample-clock differences between independent audio devices.
- Handshake messages and empty link polls use a compact independently coded
  frame, while frames carrying IP fragments use the full payload format.
- Rate-1/2 convolutional error correction and CRC-32 reject damaged frames.
- Short, numbered link transactions provide bounded retransmission for all IP
  protocols, including UDP and ICMP—not only TCP.
- IP packets up to the 1280-byte tunnel MTU are fragmented across audio frames
  and reassembled before being written to the peer's tunnel.
- Client polling makes the half-duplex acoustic channel collision-free and also
  acts as the heartbeat. Four missed responses drop the link and restart
  discovery. A short turnaround guard and receive muting during local output
  keep each machine from decoding its own speaker echo as a peer frame.

The link is designed for reliability and simplicity, not broadband speed.
Actual throughput and error rate depend heavily on the microphones, speakers,
room acoustics, volume, and selected band.

## Network layout

The automatic configuration uses a point-to-point `10.77.0.0/30` IPv4 network
and `fd77::/126` IPv6 network. Two half-default routes (`0.0.0.0/1` plus
`128.0.0.0/1`, and their IPv6 equivalents) direct client traffic into the
tunnel without deleting the machine's original default route. Closing the
tunnel removes those interface routes. Gateway firewall/NAT rules and forwarding
settings are restored during normal shutdown.

Only traffic read from the TUN/utun interface is transported. Local link-layer
protocols such as ARP are not needed because the interface is point-to-point.

## Security and current scope

The audio link is deliberately unauthenticated and unencrypted. Anyone who can
inject or receive the audio can participate in or observe the link. Use it only
in an environment where that is acceptable.

The gateway accepts one client at a time. The deterministic self-test exercises
maximum-sized packet fragmentation and both full-band and narrower-band modem
frames with added noise and multipath. It cannot substitute for testing the
chosen physical audio devices and acoustic path.

## Project clarifications

- The target is a functional, minimal, robust implementation: high-quality
  open-source-style software that stays simple rather than becoming
  hyper-defensive.
- PortAudio is permitted only if it can be included cleanly with a Makefile and
  no CMake requirement. This implementation instead uses runtime-loaded ALSA on
  Linux and built-in CoreAudio on macOS, which keeps `git clone && make`
  dependency-free.
- Networking uses TUN on Linux and utun on macOS. After automatic setup, normal
  client network traffic is routed through the gateway.
- The modem uses QPSK (4-QAM) and a 48 kHz sample rate for broad hardware
  compatibility.
- The initial implementation supports one client per gateway.
- The initial link is unauthenticated and unencrypted.
- Startup and operation log the selected input/output audio devices and useful
  connection, negotiation, retry, tunnel, and traffic information.
