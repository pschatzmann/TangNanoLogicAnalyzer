// LD_PRELOAD shim for testing only: libserialport refuses pseudo terminals
// because /sys/class/tty/pts/N does not exist. Redirect that lookup to
// ttyS0's sysfs entry so sigrok-cli can open the simulation's pty.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

static const char *fix(const char *p) {
    if (p && !strncmp(p, "/sys/class/tty/pts/", 19)) return "/sys/class/tty/ttyS0";
    return p;
}
int lstat(const char *p, struct stat *b) {
    static int (*r)(const char *, struct stat *);
    if (!r) r = dlsym(RTLD_NEXT, "lstat");
    return r(fix(p), b);
}
int fstatat(int fd, const char *p, struct stat *b, int f) {
    static int (*r)(int, const char *, struct stat *, int);
    if (!r) r = dlsym(RTLD_NEXT, "fstatat");
    return r(fd, fix(p), b, f);
}
int fstatat64(int fd, const char *p, struct stat64 *b, int f) {
    static int (*r)(int, const char *, struct stat64 *, int);
    if (!r) r = dlsym(RTLD_NEXT, "fstatat64");
    return r(fd, fix(p), b, f);
}
ssize_t readlink(const char *p, char *buf, size_t n) {
    static ssize_t (*r)(const char *, char *, size_t);
    if (!r) r = dlsym(RTLD_NEXT, "readlink");
    return r(fix(p), buf, n);
}

// Pseudo terminals have no modem control lines: pretend they exist.
#include <stdarg.h>
#include <errno.h>
#include <sys/ioctl.h>
int ioctl(int fd, unsigned long req, ...) {
    static int (*r)(int, unsigned long, ...);
    if (!r) r = dlsym(RTLD_NEXT, "ioctl");
    va_list ap; va_start(ap, req); void *arg = va_arg(ap, void *); va_end(ap);
    int ret = r(fd, req, arg);
    if (ret == -1 && errno == ENOTTY &&
        (req == TIOCMGET || req == TIOCMSET || req == TIOCMBIS || req == TIOCMBIC)) {
        if (req == TIOCMGET) *(int *)arg = TIOCM_DTR | TIOCM_RTS | TIOCM_CTS | TIOCM_DSR;
        return 0;
    }
    return ret;
}
