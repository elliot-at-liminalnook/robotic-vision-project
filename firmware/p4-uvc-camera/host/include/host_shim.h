/* Force-included (-include) ahead of the firmware source: pulls in the system headers the firmware
 * names, then points its file calls at the stand-in video devices. */
#pragma once
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "host_idf.h"
int host_open(const char *path, int flags, ...);
int host_close(int fd);
int host_ioctl(int fd, unsigned long req, ...);
void *host_mmap(void *addr, size_t len, int prot, int flags, int fd, long offset);
int host_munmap(void *addr, size_t len);
#define open   host_open
#define close  host_close
#define ioctl  host_ioctl
#define mmap   host_mmap
#define munmap host_munmap
