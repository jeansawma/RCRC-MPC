/* Bounded newlib heap and explicit unsupported POSIX operations. No
 * semihosting traps or blocking console I/O on the real-time target. */
#include <stddef.h>
#include <stdint.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
extern char _end;
void *_sbrk(ptrdiff_t increment)
{
    static uintptr_t current;
    const uintptr_t base = (uintptr_t)&_end;
    if (!current) current = base;
    uintptr_t previous = current;
    if (increment >= 0) {
        if ((uintptr_t)increment > base + 0x1000U - current) { errno = ENOMEM; return (void *)-1; }
        current += (uintptr_t)increment;
    } else {
        uintptr_t amount = (uintptr_t)(-(increment + 1)) + 1U;
        if (amount > current - base) { errno = EINVAL; return (void *)-1; }
        current -= amount;
    }
    return (void *)previous;
}
int _close(int fd) { (void)fd; errno = EBADF; return -1; }
int _fstat(int fd, struct stat *st)
{
    if (fd < 0 || fd > 2 || !st) { errno = EBADF; return -1; }
    st->st_mode = S_IFCHR; return 0;
}
int _getpid(void) { return 1; }
int _isatty(int fd) { if (fd >= 0 && fd <= 2) return 1; errno = EBADF; return 0; }
int _kill(int pid, int signal) { (void)pid; (void)signal; errno = ENOSYS; return -1; }
off_t _lseek(int fd, off_t offset, int whence) { (void)fd; (void)offset; (void)whence; errno = ESPIPE; return (off_t)-1; }
ssize_t _read(int fd, void *buffer, size_t length) { (void)fd; (void)buffer; (void)length; errno = ENOSYS; return -1; }
ssize_t _write(int fd, const void *buffer, size_t length) { (void)fd; (void)buffer; (void)length; errno = ENOSYS; return -1; }
