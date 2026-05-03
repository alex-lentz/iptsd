# IPTSD

> **This is a personal fork of [linux-surface/iptsd](https://github.com/linux-surface/iptsd)**,
> **vibe coded** (you have been warned) to get my MS Surface Laptop 7 Intel haptic trackpad to work. It tracks the upstream project and adds a small set of patches not yet
> merged upstream. See [Changes from upstream](#changes-from-upstream) below.

This is the userspace touch processing daemon for Microsoft Surface devices using Intel Precise
Touch technology.

The daemon will read incoming HID reports containing raw capacitive touch data, stylus coordinates
and DFT pen measurements, and create standard input events from it using uinput devices.

### Changes from upstream

The following patches are applied on top of [v3.1.0](https://github.com/linux-surface/iptsd/releases/tag/v3.1.0):

- **Fix physical click on Sensel haptic touchpad** (`ipts/parser`, `daemon`) — Parses IPTS report
  frame type `0x94`, which carries touchpad button state on devices like the Surface Laptop 7
  Intel. Bit 1 of byte 2 signals a physical button press; this is now routed through the existing
  `on_button` path so `BTN_LEFT` is emitted correctly.

- **Fix iptsd not recovering after sleep when hidraw device persists** (`etc`) — Adds a
  `systemd-sleep` hook that re-fires udev `add` events for all hidraw devices on post-resume. On
  some Surface devices the hidraw node is not removed during suspend, so no udev `ACTION=="add"`
  event fires on wake and `iptsd@.service` is never restarted. The hook works around this by
  running `udevadm trigger` after resume. Also adds `Restart=on-failure` to the service unit so
  that a transient startup failure immediately after wake is retried automatically.

### Credits

IPTSD is developed and maintained by the [linux-surface](https://github.com/linux-surface) team,
primarily [Maximilian Luz](https://github.com/quo) and [Dorian Stoll](https://github.com/StollD).
All core functionality originates from the upstream project. The patches in this fork were authored
by Alex Lentz with assistance from [Claude](https://claude.ai).

### Installing

IPTSD is included in the linux-surface repository. This is the recommended way of installing it.

**Important:** Support on Debian based distributions only goes back to Debian 11 / Ubuntu 22.04.

If you want to try out changes that are not yet released, GitHub Actions builds Arch Linux, Debian
and Fedora packages for every commit. You'll need to be signed in to GitHub, then go to
https://github.com/linux-surface/iptsd/actions, select the latest successful workflow and download
the artifact named `<your distro>-latest`.

### Building

To build IPTSD from source, you need to install the following dependencies:

 * A C++ compiler
 * meson
 * ninja
 * CLI11
 * Eigen3
 * fmt
 * inih / INIReader
 * gsl
 * spdlog
 * cmake, because some of our dependencies don't ship pkgconfig files

To build the plotting tools for visualizing data, you need to install a few more dependencies.

 * cairomm
 * SDL2

Most of the dependencies can be downloaded and included automatically by meson, should your
distribution not include them.

```bash
$ git clone https://github.com/linux-surface/iptsd
$ cd iptsd
$ meson setup build
$ ninja -C build
```

To run iptsd, you need to determine the ID of the hidraw device of your touchscreen:

```bash
$ sudo ./etc/iptsd-find-hidraw
```

You can then run iptsd with the device path as a launch argument:

```bash
$ sudo ./build/src/iptsd /dev/hidrawN
```

Alternatively, you can install the files you just built to the system. After a reboot, iptsd will
get started by udev automatically:

```bash
$ sudo ninja -C build install
```

On Fedora (or any other SELinux enabled distribution) you also need to fix the SELinux contexts:

```bash
$ sudo semanage fcontext -a -t systemd_unit_file_t -s system_u /usr/lib/systemd/system/iptsd@.service
$ sudo semanage fcontext -a -t usr_t -s system_u '/usr/local/bin/ipts.*'

$ sudo restorecon -vF /usr/lib/systemd/system/iptsd@.service
$ sudo restorecon -vF /usr/local/bin/ipts*
```

This is only necessary when using `ninja install`. When you install one of the packages from
GitHub Actions, or build your own package, everything will just work.
