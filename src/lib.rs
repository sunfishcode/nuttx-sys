//! Raw bindings to NuttX APIs.
//!
//! The functions in this file were translated from syscall/syscall.csv,
//! libs/libc/math.csv, and libs/libc/libc.csv in the [incubator-nuttx] repo.
//! The types were translated from headers in the include directory.
//!
//! [incubator-nuttx]: https://github.com/apache/incubator-nuttx

#![no_std]
#![allow(non_camel_case_types)]
#![feature(c_variadic)]

pub use core::ffi::{
    c_char, c_int, c_long, c_schar, c_short, c_uchar, c_uint, c_ulong, c_ushort, c_void,
    VaList as va_list,
};
#[cfg(feature = "have_long_long")]
pub use core::ffi::{c_longlong, c_ulonglong};

pub type time_t = u32;
pub type clockid_t = u8;
pub type timer_t = *mut c_void;

#[repr(C)]
pub struct timespec {
    pub tv_sec: time_t,
    pub tv_nsec: c_long,
}

#[repr(C)]
pub struct stat {
    pub st_dev: dev_t,
    pub st_ino: ino_t,
    pub st_mode: mode_t,
    pub st_nlink: nlink_t,
    pub st_uid: uid_t,
    pub st_gid: gid_t,
    pub st_rdev: dev_t,
    pub st_size: off_t,
    pub st_atim: timespec,
    pub st_mtim: timespec,
    pub st_ctim: timespec,
    pub st_blksize: blksize_t,
    pub st_blocks: blkcnt_t,
}

#[repr(C)]
pub struct statfs {
    pub f_type: u32,
    pub f_namelen: size_t,
    pub f_bsize: size_t,
    pub f_blocks: fsblkcnt_t,
    pub f_bfree: fsblkcnt_t,
    pub f_bavail: fsblkcnt_t,
    pub f_files: fsfilcnt_t,
    pub f_ffree: fsfilcnt_t,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct iovec {
    pub iov_base: *mut c_void,
    pub iov_len: size_t,
}

pub type nfds_t = c_uint;
pub type pollevent_t = u32;

#[repr(C)]
#[derive(Debug, Clone)]
pub struct pollfd {
    pub fd: c_int,
    pub events: pollevent_t,
    pub revents: pollevent_t,
    pub ptr: *mut c_void,
    pub sem: sem_t,
    pub r#priv: *mut c_void,
}

/// These types need to be ported from the nuttx headers.
pub type FIXME = c_void;

pub type timezone = FIXME;
pub type mbstate_t = FIXME;
pub type tm = FIXME;
pub type uu16 = FIXME;
pub type utsname = FIXME;
pub type sq_queue_t = FIXME;
pub type sq_entry_t = FIXME;
pub type sigset_t = FIXME;
pub type intmax_t = FIXME;
pub type uintmax_t = FIXME;
pub type dirent = FIXME;
pub type dq_entry_t = FIXME;
pub type dq_queue_t = FIXME;
pub type ether_addr = FIXME;
pub type fd_set = FIXME;
pub type itimerspec = FIXME;
pub type itimerval = FIXME;
pub type main_t = FIXME;
pub type mq_attr = FIXME;
pub type mqd_t = FIXME;
pub type posix_spawnattr_t = FIXME;
pub type posix_spawn_file_actions_t = FIXME;
pub type _sa_handler_t = FIXME;
pub type sched_param = FIXME;
pub type sigevent = FIXME;
pub type siginfo = FIXME;
pub type stackinfo_s = FIXME;
pub type symtab_s = FIXME;
pub type timeval = FIXME;
pub type sigaction = FIXME;
pub type sysinfo = FIXME;
pub type mallinfo = FIXME;
pub type pthread_t = FIXME;
pub type pthread_once_t = FIXME;
pub type pthread_mutexattr_t = FIXME;
pub type pthread_mutex_t = FIXME;
pub type pthread_startroutine_t = FIXME;
pub type pthread_attr_t = FIXME;
pub type pthread_condattr_t = FIXME;
pub type pthread_cond_t = FIXME;
pub type pthread_addr_t = FIXME;
pub type pthread_barrierattr_t = FIXME;
pub type pthread_barrier_t = FIXME;
pub type pthread_trampoline_t = FIXME;

pub type FILE = c_void;
pub type DIR = c_void;

pub type in_addr_t = u32;

#[cfg(all(feature = "fs_largefile", feature = "have_long_long"))]
pub type fsblkcnt64_t = fsblkcnt_t;
#[cfg(all(feature = "fs_largefile", feature = "have_long_long"))]
pub type fsfilcnt64_t = fsfilcnt_t;
#[cfg(all(feature = "fs_largefile", feature = "have_long_long"))]
pub type blkcnt64_t = blkcnt_t;
#[cfg(all(feature = "fs_largefile", feature = "have_long_long"))]
pub type off64_t = off_t;
#[cfg(all(feature = "fs_largefile", feature = "have_long_long"))]
pub type fpos64_t = fpos_t;

pub type mode_t = c_uint;

#[cfg(feature = "small_memory")]
pub type size_t = u16;
#[cfg(feature = "small_memory")]
pub type ssize_t = i16;
#[cfg(feature = "small_memory")]
pub type rsize_t = u16;

// FIXME: These should be `size_t`/`ssize_t`, but technically `usize`/`isize`
// correspond to `uintptr_t`/`intptr_t`.
#[cfg(not(feature = "small_memory"))]
pub type size_t = usize;
#[cfg(not(feature = "small_memory"))]
pub type ssize_t = isize;
#[cfg(not(feature = "small_memory"))]
pub type rsize_t = usize;

pub type uid_t = i16;
pub type gid_t = i16;
pub type dev_t = u32;
pub type ino_t = u16;
pub type nlink_t = u16;
pub type pid_t = c_int;
pub type id_t = c_int;
pub type key_t = i32;
pub type ptrdiff_t = isize;

// This appears to be how `wchar_t` is defined in newlibc in the past.
pub type wchar_t = u16;

pub type wint_t = c_int;
pub type wctype_t = c_int;

#[cfg(all(feature = "fs_largefile", feature = "have_long_long"))]
pub type fsblkcnt_t = u64;
#[cfg(all(feature = "fs_largefile", feature = "have_long_long"))]
pub type fsfilcnt_t = u64;
#[cfg(all(feature = "fs_largefile", feature = "have_long_long"))]
pub type blkcnt_t = u64;
#[cfg(all(feature = "fs_largefile", feature = "have_long_long"))]
pub type off_t = i64;
#[cfg(all(feature = "fs_largefile", feature = "have_long_long"))]
pub type fpos_t = i64;

#[cfg(not(all(feature = "fs_largefile", feature = "have_long_long")))]
pub type fsblkcnt_t = u32;
#[cfg(not(all(feature = "fs_largefile", feature = "have_long_long")))]
pub type fsfilcnt_t = u32;
#[cfg(not(all(feature = "fs_largefile", feature = "have_long_long")))]
pub type blkcnt_t = u32;
#[cfg(not(all(feature = "fs_largefile", feature = "have_long_long")))]
pub type off_t = i32;
#[cfg(not(all(feature = "fs_largefile", feature = "have_long_long")))]
pub type fpos_t = i32;

pub type blksize_t = i16;

pub type socklen_t = c_uint;
pub type sa_family_t = u16;

#[cfg(feature = "system_time64")]
pub type clock_t = u64;
#[cfg(not(feature = "system_time64"))]
pub type clock_t = u32;

pub type useconds_t = u32;
pub type suseconds_t = i32;

#[cfg(feature = "smp_ncpus_8")]
pub type cpu_set_t = u8;
#[cfg(feature = "smp_ncpus_16")]
pub type cpu_set_t = u16;
#[cfg(feature = "smp_ncpus_32")]
pub type cpu_set_t = u32;

#[repr(C)]
#[derive(Debug, Clone)]
pub struct sem_t {
    semcount: i16, // FIXME: volatile!?

    #[cfg(feature = "priority_inheritance")]
    flags: u8,

    #[cfg(feature = "priority_inheritance")]
    #[cfg(feature = "sem_preallocholders")] // FIXME: > 0
    hhead: semholder_t,

    #[cfg(feature = "priority_inheritance")]
    #[cfg(not(feature = "sem_preallocholders"))] // FIXME: > 0
    holder: [semholder_t; 2],
}

#[cfg(feature = "net")]
#[repr(C)]
pub struct sockaddr_storage {
    pub ss_family: sa_family_t,
    pub ss_data: [c_char; 126],
}

#[cfg(feature = "net")]
#[repr(C)]
pub struct sockaddr {
    pub sa_family: sa_family_t,
    pub sa_data: [c_char; 14],
}

#[cfg(feature = "net")]
#[repr(C)]
pub struct linger {
    pub l_onoff: c_int,
    pub l_linger: c_int,
}

#[cfg(feature = "net")]
#[repr(C)]
pub struct msghdr {
    pub msg_name: *mut c_void,
    pub msg_namelen: socklen_t,
    pub msg_iov: *mut iovec,
    pub msg_iovlen: c_ulong,
    pub msg_control: *mut c_void,
    pub msg_controllen: c_ulong,
    pub msg_flags: c_uint,
}

#[cfg(feature = "net")]
#[repr(C)]
pub struct cmsghdr {
    pub cmsg_len: c_ulong,
    pub cmsg_level: c_int,
    pub cmsg_type: c_int,
}

extern "C" {
    pub fn _exit(a1: c_int) -> !;
    #[cfg(feature = "net")]
    pub fn accept(a1: c_int, a2: *mut sockaddr, a3: *mut socklen_t) -> c_int;
    #[cfg(feature = "clock_timekeeping")]
    pub fn adjtime(a1: *const timeval, a2: *mut timeval) -> c_int;
    #[cfg(feature = "fs_aio")]
    pub fn aio_cancel(a1: c_int, a2: *mut aiocb) -> c_int;
    #[cfg(feature = "fs_aio")]
    pub fn aio_fsync(a1: c_int, a2: *mut aiocb) -> c_int;
    #[cfg(feature = "fs_aio")]
    pub fn aio_read(a1: *mut aiocb) -> c_int;
    #[cfg(feature = "fs_aio")]
    pub fn aio_write(a1: *mut aiocb) -> c_int;
    #[cfg(feature = "crypto_random_pool")]
    pub fn arc4random_buf(a1: *mut c_void, a2: size_t) -> c_void;
    #[cfg(feature = "net")]
    pub fn bind(a1: c_int, a2: *const sockaddr, a3: socklen_t) -> c_int;
    #[cfg(feature = "boardctl")]
    pub fn boardctl(a1: c_uint, a2: usize) -> c_int;
    pub fn chmod(a1: *const c_char, a2: mode_t) -> c_int;
    pub fn chown(a1: *const c_char, a2: uid_t, a3: gid_t) -> c_int;
    #[cfg(feature = "environ")]
    pub fn clearenv() -> c_int;
    pub fn clock() -> clock_t;
    pub fn clock_getres(a1: clockid_t, a2: *mut timespec) -> c_int;
    pub fn clock_gettime(a1: clockid_t, a2: *mut timespec) -> c_int;
    pub fn clock_nanosleep(
        a1: clockid_t,
        a2: c_int,
        a3: *const timespec,
        a4: *mut timespec,
    ) -> c_int;
    pub fn clock_settime(a1: clockid_t, a2: *const timespec) -> c_int;
    pub fn close(a1: c_int) -> c_int;
    #[cfg(feature = "net")]
    pub fn connect(a1: c_int, a2: *const sockaddr, a3: socklen_t) -> c_int;
    pub fn dup(a1: c_int) -> c_int;
    pub fn dup2(a1: c_int, a2: c_int) -> c_int;
    #[cfg(feature = "event_fd")]
    pub fn eventfd(a1: c_uint, a2: c_int) -> c_int;
    #[cfg(all(feature = "binfmt", not(feature = "build_kernel")))]
    pub fn exec(
        a1: *const c_char,
        a2: *mut *const c_char,
        a3: *mut *const c_char,
        a4: *const symtab_s,
        a5: c_int,
    ) -> c_int;
    #[cfg(all(not(feature = "binfmt"), feature = "libc_execfunc"))]
    pub fn execve(a1: *const c_char, a2: *mut *const c_char, a3: *mut *const c_char) -> c_int;
    pub fn fchmod(a1: c_int, a2: mode_t) -> c_int;
    pub fn fchown(a1: c_int, a2: uid_t, a3: gid_t) -> c_int;
    pub fn fcntl(a1: c_int, a2: c_int, a3: ...) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fs_fdopen(a1: c_int, a2: c_int, a3: *mut tcb_s, a4: *mut *mut file_struct) -> c_int;
    pub fn fstat(a1: c_int, a2: *mut stat) -> c_int;
    pub fn fstatfs(a1: c_int, a2: *mut statfs) -> c_int;
    #[cfg(feature = "mountpoint")]
    pub fn fsync(a1: c_int) -> c_int;
    #[cfg(feature = "mountpoint")]
    pub fn ftruncate(a1: c_int, a2: off_t) -> c_int;
    pub fn futimens(a1: c_int, a2: *const timespec) -> c_int;
    #[cfg(feature = "environ")]
    pub fn get_environ_ptr() -> *mut *mut c_char;
    #[cfg(feature = "environ")]
    pub fn getenv(a1: *const c_char) -> *mut c_char;
    #[cfg(feature = "sched_user_identity")]
    pub fn getgid() -> gid_t;
    pub fn gethostname(a1: *mut c_char, a2: size_t) -> c_int;
    #[cfg(feature = "posix_timers")]
    pub fn getitimer(a1: c_int, a2: *mut itimerval) -> c_int;
    #[cfg(feature = "net")]
    pub fn getpeername(a1: c_int, a2: *mut sockaddr, a3: *mut socklen_t) -> c_int;
    pub fn getpid() -> pid_t;
    #[cfg(feature = "sched_have_parent")]
    pub fn getppid() -> pid_t;
    #[cfg(feature = "net")]
    pub fn getsockname(a1: c_int, a2: *mut sockaddr, a3: *mut socklen_t) -> c_int;
    #[cfg(feature = "net")]
    pub fn getsockopt(
        a1: c_int,
        a2: c_int,
        a3: c_int,
        a4: *mut c_void,
        a5: *mut socklen_t,
    ) -> c_int;
    pub fn gettid() -> pid_t;
    #[cfg(feature = "sched_user_identity")]
    pub fn getuid() -> uid_t;
    #[cfg(feature = "module")]
    pub fn insmod(a1: *const c_char, a2: *const c_char) -> *mut c_void;
    pub fn ioctl(a1: c_int, a2: c_int, a3: ...) -> c_int;
    pub fn kill(a1: pid_t, a2: c_int) -> c_int;
    pub fn lchmod(a1: *const c_char, a2: mode_t) -> c_int;
    pub fn lchown(a1: *const c_char, a2: uid_t, a3: gid_t) -> c_int;
    #[cfg(feature = "net")]
    pub fn listen(a1: c_int, a2: c_int) -> c_int;
    pub fn lseek(a1: c_int, a2: off_t, a3: c_int) -> off_t;
    pub fn lstat(a1: *const c_char, a2: *mut stat) -> c_int;
    pub fn lutimens(a1: *const c_char, a2: *const timespec) -> c_int;
    #[cfg(feature = "mountpoint")]
    pub fn mkdir(a1: *const c_char, a2: mode_t) -> c_int;
    pub fn mmap(
        a1: *mut c_void,
        a2: size_t,
        a3: c_int,
        a4: c_int,
        a5: c_int,
        a6: off_t,
    ) -> *mut c_void;
    #[cfg(feature = "module")]
    pub fn modhandle(a1: *const c_char) -> *mut c_void;
    #[cfg(feature = "mountpoint")]
    pub fn mount(
        a1: *const c_char,
        a2: *const c_char,
        a3: *const c_char,
        a4: c_ulong,
        a5: *const c_void,
    ) -> c_int;
    #[cfg(feature = "mqueue")]
    pub fn mq_close(a1: mqd_t) -> c_int;
    #[cfg(feature = "mqueue")]
    pub fn mq_getattr(a1: mqd_t, a2: *mut mq_attr) -> c_int;
    #[cfg(feature = "mqueue")]
    pub fn mq_notify(a1: mqd_t, a2: *const sigevent) -> c_int;
    #[cfg(feature = "mqueue")]
    pub fn mq_open(a1: *const c_char, a2: c_int, a3: ...) -> mqd_t;
    #[cfg(feature = "mqueue")]
    pub fn mq_receive(a1: mqd_t, a2: *mut c_char, a3: size_t, a4: *mut c_uint) -> ssize_t;
    #[cfg(feature = "mqueue")]
    pub fn mq_send(a1: mqd_t, a2: *const c_char, a3: size_t, a4: c_uint) -> c_int;
    #[cfg(feature = "mqueue")]
    pub fn mq_setattr(a1: mqd_t, a2: *const mq_attr, a3: *mut mq_attr) -> c_int;
    #[cfg(feature = "mqueue")]
    pub fn mq_timedreceive(
        a1: mqd_t,
        a2: *mut c_char,
        a3: size_t,
        a4: *mut c_uint,
        a5: *const timespec,
    ) -> ssize_t;
    #[cfg(feature = "mqueue")]
    pub fn mq_timedsend(
        a1: mqd_t,
        a2: *const c_char,
        a3: size_t,
        a4: c_uint,
        a5: *const timespec,
    ) -> c_int;
    #[cfg(feature = "mqueue")]
    pub fn mq_unlink(a1: *const c_char) -> c_int;
    #[cfg(feature = "fs_rammap")]
    pub fn munmap(a1: *mut c_void, a2: size_t) -> c_int;
    #[cfg(all(feature = "pipes", feature = "dev_fifo"))]
    pub fn nx_mkfifo(a1: *const c_char, a2: mode_t, a3: size_t) -> c_int;
    #[cfg(all(feature = "pipes", feature = "dev_pipe"))]
    pub fn nx_pipe(a1: *mut c_int, a2: size_t, a3: c_int) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn nx_pthread_create(
        a1: pthread_trampoline_t,
        a2: *mut pthread_t,
        a3: *const pthread_attr_t,
        a4: pthread_startroutine_t,
        a5: pthread_addr_t,
    ) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn nx_pthread_exit(a1: pthread_addr_t) -> !;
    pub fn nx_vsyslog(a1: c_int, a2: *const c_char, a3: *mut va_list) -> c_int;
    pub fn nxsched_get_stackinfo(a1: pid_t, a2: *mut stackinfo_s) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn nxsched_get_streams() -> *mut streamlist;
    pub fn open(a1: *const c_char, a2: c_int, a3: ...) -> c_int;
    #[cfg(feature = "build_kernel")]
    pub fn pgalloc(a1: usize, a2: c_uint) -> usize;
    pub fn poll(a1: *mut pollfd, a2: nfds_t, a3: c_int) -> c_int;
    #[cfg(all(not(feature = "binfmt"), feature = "libc_execfunc"))]
    pub fn posix_spawn(
        a1: *mut pid_t,
        a2: *const c_char,
        a3: *const posix_spawn_file_actions_t,
        a4: *const posix_spawnattr_t,
        a5: *mut *const c_char,
        a6: *mut *const c_char,
    ) -> c_int;
    pub fn ppoll(a1: *mut pollfd, a2: nfds_t, a3: *const timespec, a4: *const sigset_t) -> c_int;
    #[cfg(feature = "task_name")]
    pub fn prctl(a1: c_int, a2: ...) -> c_int;
    pub fn pread(a1: c_int, a2: *mut c_void, a3: size_t, a4: off_t) -> ssize_t;
    pub fn pselect(
        a1: c_int,
        a2: *mut fd_set,
        a3: *mut fd_set,
        a4: *mut fd_set,
        a5: *const timespec,
        a6: *const sigset_t,
    ) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_cancel(a1: pthread_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_cond_broadcast(a1: *mut pthread_cond_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_cond_clockwait(
        a1: *mut pthread_cond_t,
        a2: *mut pthread_mutex_t,
        a3: clockid_t,
        a4: *const timespec,
    ) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_cond_signal(a1: *mut pthread_cond_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_cond_wait(a1: *mut pthread_cond_t, a2: *mut pthread_mutex_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_detach(a1: pthread_t) -> c_int;
    #[cfg(all(feature = "pthread", feature = "smp"))]
    pub fn pthread_getaffinity_np(a1: pthread_t, a2: size_t, a3: *mut cpu_set_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_getschedparam(a1: pthread_t, a2: *mut c_int, a3: *mut sched_param) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_join(a1: pthread_t, a2: *mut pthread_addr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_kill(a1: pthread_t, a2: c_int) -> c_int;
    #[cfg(all(feature = "pthread", not(feature = "pthread_mutex_unsafe")))]
    pub fn pthread_mutex_consistent(a1: *mut pthread_mutex_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_mutex_destroy(a1: *mut pthread_mutex_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_mutex_init(a1: *mut pthread_mutex_t, a2: *const pthread_mutexattr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_mutex_timedlock(a1: *mut pthread_mutex_t, a2: *const timespec) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_mutex_trylock(a1: *mut pthread_mutex_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_mutex_unlock(a1: *mut pthread_mutex_t) -> c_int;
    #[cfg(all(feature = "pthread", feature = "smp"))]
    pub fn pthread_setaffinity_np(a1: pthread_t, a2: size_t, a3: *const cpu_set_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_setschedparam(a1: pthread_t, a2: c_int, a3: *const sched_param) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_setschedprio(a1: pthread_t, a2: c_int) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_sigmask(a1: c_int, a2: *const sigset_t, a3: *mut sigset_t) -> c_int;
    #[cfg(feature = "environ")]
    pub fn putenv(a1: *const c_char) -> c_int;
    pub fn pwrite(a1: c_int, a2: *const c_void, a3: size_t, a4: off_t) -> ssize_t;
    pub fn read(a1: c_int, a2: *mut c_void, a3: size_t) -> ssize_t;
    #[cfg(feature = "pseudofs_softlinks")]
    pub fn readlink(a1: *const c_char, a2: *mut c_char, a3: size_t) -> ssize_t;
    #[cfg(feature = "net")]
    pub fn recv(a1: c_int, a2: *mut c_void, a3: size_t, a4: c_int) -> ssize_t;
    #[cfg(feature = "net")]
    pub fn recvfrom(
        a1: c_int,
        a2: *mut c_void,
        a3: size_t,
        a4: c_int,
        a5: *mut sockaddr,
        a6: *mut socklen_t,
    ) -> ssize_t;
    #[cfg(feature = "net")]
    pub fn recvmsg(a1: c_int, a2: *mut msghdr, a3: c_int) -> ssize_t;
    #[cfg(feature = "mountpoint")]
    pub fn rename(a1: *const c_char, a2: *const c_char) -> c_int;
    #[cfg(feature = "mountpoint")]
    pub fn rmdir(a1: *const c_char) -> c_int;
    #[cfg(feature = "module")]
    pub fn rmmod(a1: *mut c_void) -> c_int;
    #[cfg(feature = "smp")]
    pub fn sched_getaffinity(a1: pid_t, a2: size_t, a3: *mut cpu_set_t) -> c_int;
    #[cfg(feature = "smp")]
    pub fn sched_getcpu() -> c_int;
    pub fn sched_getparam(a1: pid_t, a2: *mut sched_param) -> c_int;
    pub fn sched_getscheduler(a1: pid_t) -> c_int;
    pub fn sched_lock() -> c_int;
    pub fn sched_lockcount() -> c_int;
    pub fn sched_rr_get_interval(a1: pid_t, a2: *mut timespec) -> c_int;
    #[cfg(feature = "smp")]
    pub fn sched_setaffinity(a1: pid_t, a2: size_t, a3: *const cpu_set_t) -> c_int;
    pub fn sched_setparam(a1: pid_t, a2: *const sched_param) -> c_int;
    pub fn sched_setscheduler(a1: pid_t, a2: c_int, a3: *const sched_param) -> c_int;
    pub fn sched_unlock() -> c_int;
    pub fn sched_yield() -> c_int;
    #[cfg(feature = "sched_backtrace")]
    pub fn sched_backtrace(a1: pid_t, a2: *mut *mut c_void, a3: c_int, a4: c_int) -> c_int;
    pub fn select(
        a1: c_int,
        a2: *mut fd_set,
        a3: *mut fd_set,
        a4: *mut fd_set,
        a5: *mut timeval,
    ) -> c_int;
    pub fn sem_clockwait(a1: *mut sem_t, a2: clockid_t, a3: *const timespec) -> c_int;
    #[cfg(feature = "fs_named_semaphores")]
    pub fn sem_close(a1: *mut sem_t) -> c_int;
    pub fn sem_destroy(a1: *mut sem_t) -> c_int;
    #[cfg(feature = "fs_named_semaphores")]
    pub fn sem_open(a1: *const c_char, a2: c_int, a3: ...) -> *mut sem_t;
    pub fn sem_post(a1: *mut sem_t) -> c_int;
    #[cfg(feature = "priority_inheritance")]
    pub fn sem_setprotocol(a1: *mut sem_t, a2: c_int) -> c_int;
    pub fn sem_timedwait(a1: *mut sem_t, a2: *const timespec) -> c_int;
    pub fn sem_trywait(a1: *mut sem_t) -> c_int;
    #[cfg(feature = "fs_named_semaphores")]
    pub fn sem_unlink(a1: *const c_char) -> c_int;
    pub fn sem_wait(a1: *mut sem_t) -> c_int;
    #[cfg(feature = "net")]
    pub fn send(a1: c_int, a2: *const c_void, a3: size_t, a4: c_int) -> ssize_t;
    pub fn sendfile(a1: c_int, a2: c_int, a3: *mut off_t, a4: size_t) -> ssize_t;
    #[cfg(feature = "net")]
    pub fn sendmsg(a1: c_int, a2: *mut msghdr, a3: c_int) -> ssize_t;
    #[cfg(feature = "net")]
    pub fn sendto(
        a1: c_int,
        a2: *const c_void,
        a3: size_t,
        a4: c_int,
        a5: *const sockaddr,
        a6: socklen_t,
    ) -> ssize_t;
    #[cfg(feature = "environ")]
    pub fn setenv(a1: *const c_char, a2: *const c_char, a3: c_int) -> c_int;
    #[cfg(feature = "sched_user_identity")]
    pub fn setgid(a1: gid_t) -> c_int;
    pub fn sethostname(a1: *const c_char, a2: size_t) -> c_int;
    #[cfg(feature = "posix_timers")]
    pub fn setitimer(a1: c_int, a2: *const itimerval, a3: *mut itimerval) -> c_int;
    #[cfg(feature = "net")]
    pub fn setsockopt(a1: c_int, a2: c_int, a3: c_int, a4: *const c_void, a5: socklen_t) -> c_int;
    #[cfg(feature = "sched_user_identity")]
    pub fn setuid(a1: uid_t) -> c_int;
    #[cfg(feature = "mm_shm")]
    pub fn shmat(a1: c_int, a2: *const c_void, a3: c_int) -> *mut c_void;
    #[cfg(feature = "mm_shm")]
    pub fn shmctl(a1: c_int, a2: c_int, a3: *mut shmid_ds) -> c_int;
    #[cfg(feature = "mm_shm")]
    pub fn shmdt(a1: *const c_void) -> c_int;
    #[cfg(feature = "mm_shm")]
    pub fn shmget(a1: key_t, a2: size_t, a3: c_int) -> c_int;
    pub fn sigaction(a1: c_int, a2: *const sigaction, a3: *mut sigaction) -> c_int;
    pub fn sigpending(a1: *mut sigset_t) -> c_int;
    pub fn sigprocmask(a1: c_int, a2: *const sigset_t, a3: *mut sigset_t) -> c_int;
    pub fn sigqueue(a1: c_int, a2: c_int, a3: *mut c_void) -> c_int;
    pub fn sigsuspend(a1: *const sigset_t) -> c_int;
    pub fn sigtimedwait(a1: *const sigset_t, a2: *mut siginfo, a3: *const timespec) -> c_int;
    pub fn sigwaitinfo(a1: *const sigset_t, a2: *mut siginfo) -> c_int;
    #[cfg(feature = "net")]
    pub fn socket(a1: c_int, a2: c_int, a3: c_int) -> c_int;
    #[cfg(feature = "net")]
    pub fn socketpair(a1: c_int, a2: c_int, a3: c_int, a4: *mut c_int) -> c_int;
    pub fn stat(a1: *const c_char, a2: *mut stat) -> c_int;
    pub fn statfs(a1: *const c_char, a2: *mut statfs) -> c_int;
    #[cfg(feature = "pseudofs_softlinks")]
    pub fn symlink(a1: *const c_char, a2: *const c_char) -> c_int;
    pub fn sysinfo(a1: *mut sysinfo) -> c_int;
    #[cfg(not(feature = "build_kernel"))]
    pub fn task_create(
        a1: *const c_char,
        a2: c_int,
        a3: c_int,
        a4: main_t,
        a5: *mut *const c_char,
    ) -> c_int;
    #[cfg(not(feature = "build_kernel"))]
    pub fn task_delete(a1: pid_t) -> c_int;
    #[cfg(not(feature = "build_kernel"))]
    pub fn task_restart(a1: pid_t) -> c_int;
    pub fn task_setcancelstate(a1: c_int, a2: *mut c_int) -> c_int;
    #[cfg(feature = "cancellation_points")]
    pub fn task_setcanceltype(a1: c_int, a2: *mut c_int) -> c_int;
    #[cfg(not(feature = "build_kernel"))]
    pub fn task_spawn(
        a1: *const c_char,
        a2: main_t,
        a3: *const posix_spawn_file_actions_t,
        a4: *const posix_spawnattr_t,
        a5: *mut *const c_char,
        a6: *mut *const c_char,
    ) -> c_int;
    #[cfg(feature = "cancellation_points")]
    pub fn task_testcancel() -> c_void;
    #[cfg(feature = "tls_task")]
    pub fn task_tls_alloc(a1: tls_dtor_t) -> c_int;
    #[cfg(feature = "posix_timers")]
    pub fn timer_create(a1: clockid_t, a2: *mut sigevent, a3: *mut timer_t) -> c_int;
    #[cfg(feature = "posix_timers")]
    pub fn timer_delete(a1: timer_t) -> c_int;
    #[cfg(feature = "posix_timers")]
    pub fn timer_getoverrun(a1: timer_t) -> c_int;
    #[cfg(feature = "posix_timers")]
    pub fn timer_gettime(a1: timer_t, a2: *mut itimerspec) -> c_int;
    #[cfg(feature = "posix_timers")]
    pub fn timer_settime(
        a1: timer_t,
        a2: c_int,
        a3: *const itimerspec,
        a4: *mut itimerspec,
    ) -> c_int;
    #[cfg(feature = "timer_fd")]
    pub fn timerfd_create(a1: c_int, a2: c_int) -> c_int;
    #[cfg(feature = "timer_fd")]
    pub fn timerfd_gettime(a1: c_int, a2: *mut itimerspec) -> c_int;
    #[cfg(feature = "timer_fd")]
    pub fn timerfd_settime(
        a1: c_int,
        a2: c_int,
        a3: *const itimerspec,
        a4: *const itimerspec,
    ) -> c_int;
    #[cfg(feature = "mountpoint")]
    pub fn umount2(a1: *const c_char, a2: c_uint) -> c_int;
    #[cfg(feature = "mountpoint")]
    pub fn unlink(a1: *const c_char) -> c_int;
    #[cfg(feature = "environ")]
    pub fn unsetenv(a1: *const c_char) -> c_int;
    pub fn up_assert(a1: *const c_char, a2: c_int) -> c_void;
    pub fn utimens(a1: *const c_char, a2: *const timespec) -> c_int;
    #[cfg(all(feature = "sched_waitpid", feature = "arch_have_fork"))]
    pub fn vfork() -> pid_t;
    #[cfg(all(feature = "sched_waitpid", feature = "sched_have_parent"))]
    pub fn wait(a1: *mut c_int) -> pid_t;
    #[cfg(all(feature = "sched_waitpid", feature = "sched_have_parent"))]
    pub fn waitid(a1: idtype_t, a2: id_t, a3: *mut siginfo_t, a4: c_int) -> c_int;
    #[cfg(feature = "sched_waitpid")]
    pub fn waitpid(a1: pid_t, a2: *mut c_int, a3: c_int) -> pid_t;
    pub fn write(a1: c_int, a2: *const c_void, a3: size_t) -> ssize_t;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn acos(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn acosf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn acosl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn asin(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn asinf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn asinl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn atan(a1: f64) -> f64;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn atan2(a1: f64, a2: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn atan2f(a1: f32, a2: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn atan2l(a1: c_ldouble, a2: c_ldouble) -> c_ldouble;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn atanf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn atanl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn ceil(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn ceilf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn ceill(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn cos(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn cosf(a1: f32) -> f32;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn cosh(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn coshf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn coshl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn cosl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn exp(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn expf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn expl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn fabs(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn fabsf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn fabsl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn floor(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn floorf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn floorl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn fmod(a1: f64, a2: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn fmodf(a1: f32, a2: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn fmodl(a1: c_ldouble, a2: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn frexp(a1: f64, a2: *mut c_int) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn frexpf(a1: f32, a2: *mut c_int) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn frexpl(a1: c_ldouble, a2: *mut c_int) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn gamma(a1: f64) -> f64;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn ldexp(a1: f64, a2: c_int) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn ldexpf(a1: f32, a2: c_int) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn ldexpl(a1: c_ldouble, a2: c_int) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn lgamma(a1: f64) -> f64;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn log(a1: f64) -> f64;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn log10(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn log10f(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn log10l(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn log2(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn log2f(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn log2l(a1: c_ldouble) -> c_ldouble;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn logf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn logl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn modf(a1: f64, a2: *mut f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn modff(a1: f32, a2: *mut f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn modfl(a1: c_ldouble, a2: *mut c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn pow(a1: f64, a2: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn powf(a1: f32, a2: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn powl(a1: c_ldouble, a2: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn rint(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn rintf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn rintl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn round(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn roundf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn roundl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn sin(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn sinf(a1: f32) -> f32;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn sinh(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn sinhf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn sinhl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn sinl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn sqrt(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn sqrtf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn sqrtl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn tan(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn tanf(a1: f32) -> f32;
    #[cfg(all(feature = "have_double", any(feature = "libm", feature = "arch_math")))]
    pub fn tanh(a1: f64) -> f64;
    #[cfg(any(feature = "libm", feature = "arch_math"))]
    pub fn tanhf(a1: f32) -> f32;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn tanhl(a1: c_ldouble) -> c_ldouble;
    #[cfg(all(
        feature = "have_long_double",
        any(feature = "libm", feature = "arch_math")
    ))]
    pub fn tanl(a1: c_ldouble) -> c_ldouble;
    #[cfg(feature = "build_flat")]
    pub fn __errno() -> *mut c_int;
    #[cfg(feature = "stack_canaries")]
    pub fn __stack_chk_fail(a1: c_void) -> c_void;
    #[cfg(all(not(feature = "cpp_have_varargs"), feature = "debug_error"))]
    pub fn _alert(a1: *const c_char, a2: ...) -> c_void;
    pub fn _assert(a1: *const c_char, a2: c_int) -> c_void;
    #[cfg(all(not(feature = "cpp_have_varargs"), feature = "debug_error"))]
    pub fn _err(a1: *const c_char, a2: ...) -> c_void;
    #[cfg(all(not(feature = "cpp_have_varargs"), feature = "debug_info"))]
    pub fn _info(a1: *const c_char, a2: ...) -> c_void;
    #[cfg(all(not(feature = "cpp_have_varargs"), feature = "debug_warn"))]
    pub fn _warn(a1: *const c_char, a2: ...) -> c_void;
    pub fn abort() -> c_void;
    pub fn abs(a1: c_int) -> c_int;
    #[cfg(feature = "fs_aio")]
    pub fn aio_error(a1: *mut aiocb) -> c_int;
    #[cfg(feature = "fs_aio")]
    pub fn aio_return(a1: *mut aiocb) -> ssize_t;
    #[cfg(feature = "fs_aio")]
    pub fn aio_suspend(a1: *const *const aiocb, a2: c_int, a3: *const timespec) -> c_int;
    #[cfg(feature = "posix_timers")]
    pub fn alarm(a1: c_uint) -> c_uint;
    pub fn asprintf(a1: *mut *mut c_char, a2: *const c_char, a3: ...) -> c_int;
    #[cfg(feature = "have_double")]
    pub fn atof(a1: *const c_char) -> f64;
    pub fn atoi(a1: *const c_char) -> c_int;
    pub fn atol(a1: *const c_char) -> c_long;
    #[cfg(feature = "have_long_long")]
    pub fn atoll(a1: *const c_char) -> c_longlong;
    #[cfg(not(feature = "have_long_long"))]
    pub fn b16atan2(a1: u16, a2: u16) -> u16;
    pub fn b16cos(a1: u16) -> u16;
    #[cfg(not(feature = "have_long_long"))]
    pub fn b16divb16(a1: u16, a2: u16) -> u16;
    #[cfg(not(feature = "have_long_long"))]
    pub fn b16mulb16(a1: u16, a2: u16) -> u16;
    pub fn b16sin(a1: u16) -> u16;
    #[cfg(not(feature = "have_long_long"))]
    pub fn b16sqr(a1: u16) -> u16;
    pub fn basename(a1: *mut c_char) -> *mut c_char;
    pub fn btowc(a1: c_int) -> wint_t;
    pub fn calloc(a1: size_t, a2: size_t) -> *mut c_void;
    #[cfg(feature = "serial_termios")]
    pub fn cfgetspeed(a1: *const termios) -> speed_t;
    #[cfg(feature = "serial_termios")]
    pub fn cfsetspeed(a1: *mut termios, a2: speed_t) -> c_int;
    #[cfg(feature = "environ")]
    pub fn chdir(a1: *const c_char) -> c_int;
    pub fn crc32(a1: *const u8, a2: size_t) -> u32;
    pub fn crc32part(a1: *const u8, a2: size_t, a3: u32) -> u32;
    pub fn ctime(a1: *const time_t) -> *mut c_char;
    pub fn dirname(a1: *mut c_char) -> *mut c_char;
    #[cfg(feature = "libc_dlfcn")]
    pub fn dlclose(a1: *mut c_void) -> c_int;
    #[cfg(feature = "libc_dlfcn")]
    pub fn dlerror(a1: c_void) -> *mut c_char;
    #[cfg(feature = "libc_dlfcn")]
    pub fn dlopen(a1: *const c_char, a2: c_int) -> *mut c_void;
    #[cfg(feature = "libc_dlfcn")]
    pub fn dlsym(a1: *mut c_void, a2: *const c_char) -> *mut c_void;
    #[cfg(feature = "libc_dlfcn")]
    pub fn dlsymtab(a1: *const symtab_s, a2: c_int) -> c_int;
    pub fn dq_addafter(a1: *mut dq_entry_t, a2: *mut dq_entry_t, a3: *mut dq_queue_t) -> c_void;
    pub fn dq_addbefore(a1: *mut dq_entry_t, a2: *mut dq_entry_t, a3: *mut dq_queue_t) -> c_void;
    pub fn dq_addfirst(a1: *mut dq_entry_t, a2: *mut dq_queue_t) -> c_void;
    pub fn dq_addlast(a1: *mut dq_entry_t, a2: *mut dq_queue_t) -> c_void;
    pub fn dq_rem(a1: *mut dq_entry_t, a2: *mut dq_queue_t) -> c_void;
    pub fn dq_remfirst(a1: *mut dq_queue_t) -> *mut dq_entry_t;
    pub fn dq_remlast(a1: *mut dq_queue_t) -> *mut dq_entry_t;
    pub fn ether_ntoa(a1: *const ether_addr) -> *mut c_char;
    #[cfg(feature = "file_stream")]
    pub fn fclose(a1: *mut FILE) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fdopen(a1: c_int, a2: *const c_char) -> *mut FILE;
    #[cfg(feature = "file_stream")]
    pub fn feof(a1: *mut FILE) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn ferror(a1: *mut FILE) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fflush(a1: *mut FILE) -> c_int;
    pub fn ffs(a1: c_int) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fgetc(a1: *mut FILE) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fgetpos(a1: *mut FILE, a2: *mut fpos_t) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fgets(a1: *mut c_char, a2: c_int, a3: *mut FILE) -> *mut c_char;
    pub fn fileno(a1: *mut FILE) -> c_int;
    pub fn fnmatch(a1: *const c_char, a2: *const c_char, a3: c_int) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fopen(a1: *const c_char, a2: *const c_char) -> *mut FILE;
    #[cfg(feature = "file_stream")]
    pub fn fprintf(a1: *mut FILE, a2: *const c_char, a3: ...) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fputc(a1: c_int, a2: *mut FILE) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fputs(a1: *const c_char, a2: *mut FILE) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fread(a1: *mut c_void, a2: size_t, a3: size_t, a4: *mut FILE) -> size_t;
    pub fn free(a1: *mut c_void) -> c_void;
    #[cfg(feature = "libc_netdb")]
    pub fn freeaddrinfo(a1: *mut addrinfo) -> c_void;
    #[cfg(feature = "file_stream")]
    pub fn fseek(a1: *mut FILE, a2: c_long, a3: c_int) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn fsetpos(a1: *mut FILE, a2: *mut fpos_t) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn ftell(a1: *mut FILE) -> c_long;
    #[cfg(feature = "file_stream")]
    pub fn fwrite(a1: *const c_void, a2: size_t, a3: size_t, a4: *mut FILE) -> size_t;
    #[cfg(feature = "libc_netdb")]
    pub fn gai_strerror(a1: c_int) -> *const c_char;
    #[cfg(feature = "libc_netdb")]
    pub fn getaddrinfo(
        a1: *const c_char,
        a2: *const c_char,
        a3: *const addrinfo,
        a4: *mut *mut addrinfo,
    ) -> c_int;
    #[cfg(feature = "environ")]
    pub fn getcwd(a1: *mut c_char, a2: size_t) -> *mut c_char;
    #[cfg(feature = "libc_netdb")]
    pub fn gethostbyname(a1: *const c_char) -> *mut hostent;
    #[cfg(feature = "libc_netdb")]
    pub fn gethostbyname2(a1: *const c_char, a2: c_int) -> *mut hostent;
    #[cfg(feature = "libc_netdb")]
    pub fn getnameinfo(
        a1: *const sockaddr,
        a2: socklen_t,
        a3: *mut c_char,
        a4: socklen_t,
        a5: *mut c_char,
        a6: socklen_t,
        a7: c_int,
    ) -> c_int;
    pub fn getopt(a1: c_int, a2: *mut *const c_char, a3: *const c_char) -> c_int;
    pub fn getoptargp() -> *mut *mut c_char;
    pub fn getopterrp() -> *mut c_int;
    pub fn getoptindp() -> *mut c_int;
    pub fn getoptoptp() -> *mut c_int;
    #[cfg(feature = "file_stream")]
    pub fn gets(a1: *mut c_char) -> *mut c_char;
    pub fn gettimeofday(a1: *mut timeval, a2: *mut timezone) -> c_int;
    pub fn gmtime(a1: *const time_t) -> *mut tm;
    pub fn gmtime_r(a1: *const time_t, a2: *mut tm) -> *mut tm;
    pub fn htonl(a1: u32) -> u32;
    pub fn htons(a1: u16) -> u16;
    pub fn imaxabs(a1: intmax_t) -> intmax_t;
    pub fn inet_addr(a1: *const c_char) -> in_addr_t;
    #[cfg(feature = "net_ipv4")]
    pub fn inet_ntoa(a1: in_addr) -> *mut c_char;
    pub fn inet_ntop(a1: c_int, a2: *const c_void, a3: *mut c_char, a4: socklen_t)
        -> *const c_char;
    pub fn inet_pton(a1: c_int, a2: *const c_char, a3: *mut c_void) -> c_int;
    pub fn isalnum(a1: c_int) -> c_int;
    pub fn isalpha(a1: c_int) -> c_int;
    pub fn isascii(a1: c_int) -> c_int;
    pub fn isblank(a1: c_int) -> c_int;
    pub fn iscntrl(a1: c_int) -> c_int;
    pub fn isdigit(a1: c_int) -> c_int;
    pub fn isgraph(a1: c_int) -> c_int;
    pub fn islower(a1: c_int) -> c_int;
    pub fn isprint(a1: c_int) -> c_int;
    pub fn ispunct(a1: c_int) -> c_int;
    pub fn isspace(a1: c_int) -> c_int;
    pub fn isupper(a1: c_int) -> c_int;
    pub fn iswalnum(a1: wint_t) -> c_int;
    pub fn iswalpha(a1: wint_t) -> c_int;
    pub fn iswblank(a1: wint_t) -> c_int;
    pub fn iswcntrl(a1: wint_t) -> c_int;
    pub fn iswctype(a1: wint_t, a2: wctype_t) -> c_int;
    pub fn iswdigit(a1: wint_t) -> c_int;
    pub fn iswgraph(a1: wint_t) -> c_int;
    pub fn iswlower(a1: wint_t) -> c_int;
    pub fn iswprint(a1: wint_t) -> c_int;
    pub fn iswpunct(a1: wint_t) -> c_int;
    pub fn iswspace(a1: wint_t) -> c_int;
    pub fn iswupper(a1: wint_t) -> c_int;
    pub fn iswxdigit(a1: wint_t) -> c_int;
    pub fn isxdigit(a1: c_int) -> c_int;
    pub fn labs(a1: c_long) -> c_long;
    pub fn lib_dumpbuffer(a1: *const c_char, a2: *const u8, a3: c_uint) -> c_void;
    #[cfg(feature = "fs_aio")]
    pub fn lio_listio(a1: c_int, a2: *mut *const aiocb, a3: c_int, a4: *mut sigevent) -> c_int;
    #[cfg(feature = "have_long_long")]
    pub fn llabs(a1: c_longlong) -> c_longlong;
    pub fn localtime(a1: *const time_t) -> *mut tm;
    pub fn mallinfo(a1: c_void) -> mallinfo;
    pub fn malloc(a1: size_t) -> *mut c_void;
    pub fn malloc_size(a1: *mut c_void) -> size_t;
    pub fn mblen(a1: *const c_char, a2: size_t) -> c_int;
    pub fn mbrlen(a1: *const c_char, a2: size_t, a3: *mut mbstate_t) -> size_t;
    pub fn mbrtowc(a1: *mut wchar_t, a2: *const c_char, a3: size_t, a4: *mut mbstate_t) -> size_t;
    pub fn mbsnrtowcs(
        a1: *mut wchar_t,
        a2: *const *mut c_char,
        a3: size_t,
        a4: size_t,
        a5: *mut mbstate_t,
    ) -> size_t;
    pub fn mbsrtowcs(
        a1: *mut wchar_t,
        a2: *const *mut c_char,
        a3: size_t,
        a4: *mut mbstate_t,
    ) -> size_t;
    pub fn mbstowcs(a1: *mut wchar_t, a2: *const c_char, a3: size_t) -> size_t;
    pub fn mbtowc(a1: *mut wchar_t, a2: *const c_char, a3: size_t) -> c_int;
    pub fn memccpy(a1: *mut c_void, a2: *const c_void, a3: c_int, a4: size_t) -> *mut c_void;
    pub fn memchr(a1: *const c_void, a2: c_int, a3: size_t) -> *mut c_void;
    pub fn memcmp(a1: *const c_void, a2: *const c_void, a3: size_t) -> c_int;
    pub fn memcpy(a1: *mut c_void, a2: *const c_void, a3: size_t) -> *mut c_void;
    pub fn memmove(a1: *mut c_void, a2: *const c_void, a3: size_t) -> *mut c_void;
    pub fn memset(a1: *mut c_void, a2: c_int, a3: size_t) -> *mut c_void;
    pub fn mkdtemp(a1: *mut c_char) -> *mut c_char;
    #[cfg(all(feature = "pipes", feature = "dev_fifo"))]
    pub fn mkfifo(a1: *const c_char, a2: mode_t) -> c_int;
    pub fn mkstemp(a1: *mut c_char) -> c_int;
    pub fn mktemp(a1: *mut c_char) -> *mut c_char;
    pub fn mktime(a1: *mut tm) -> time_t;
    pub fn ntohl(a1: u32) -> u32;
    pub fn ntohs(a1: u16) -> u16;
    #[cfg(feature = "file_stream")]
    pub fn perror(a1: *const c_char) -> c_void;
    #[cfg(all(feature = "pipes", feature = "dev_pipe"))]
    pub fn pipe(a1: *mut c_int) -> c_int;
    pub fn preadv(a1: c_int, a2: *const iovec, a3: c_int, a4: off_t) -> ssize_t;
    pub fn printf(a1: *const c_char, a2: ...) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_attr_destroy(a1: *mut pthread_attr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_attr_getinheritsched(a1: *const pthread_attr_t, a2: *mut c_int) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_attr_getschedparam(a1: *const pthread_attr_t, a2: *mut sched_param) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_attr_getschedpolicy(a1: *const pthread_attr_t, a2: *mut c_int) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_attr_getstacksize(a1: *const pthread_attr_t, a2: *mut size_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_attr_init(a1: *mut pthread_attr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_attr_setinheritsched(a1: *mut pthread_attr_t, a2: c_int) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_attr_setschedparam(a1: *mut pthread_attr_t, a2: *const sched_param) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_attr_setschedpolicy(a1: *mut pthread_attr_t, a2: c_int) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_attr_setstacksize(a1: *mut pthread_attr_t, a2: size_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_barrier_destroy(a1: *mut pthread_barrier_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_barrier_init(
        a1: *mut pthread_barrier_t,
        a2: *const pthread_barrierattr_t,
        a3: c_uint,
    ) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_barrier_wait(a1: *mut pthread_barrier_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_barrierattr_destroy(a1: *mut pthread_barrierattr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_barrierattr_getpshared(
        a1: *const pthread_barrierattr_t,
        a2: *mut c_int,
    ) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_barrierattr_init(a1: *mut pthread_barrierattr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_barrierattr_setpshared(a1: *mut pthread_barrierattr_t, a2: c_int) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_cond_destroy(a1: *mut pthread_cond_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_cond_init(a1: *mut pthread_cond_t, a2: *const pthread_condattr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_cond_timedwait(
        a1: *mut pthread_cond_t,
        a2: *mut pthread_mutex_t,
        a3: *const timespec,
    ) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_condattr_destroy(a1: *mut pthread_condattr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_condattr_init(a1: *mut pthread_condattr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_create(
        a1: *mut pthread_t,
        a2: *const pthread_attr_t,
        a3: pthread_startroutine_t,
        a4: pthread_addr_t,
    ) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_getname_np(a1: pthread_t, a2: *mut c_char, a3: size_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_mutex_lock(a1: *mut pthread_mutex_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_mutexattr_destroy(a1: *mut pthread_mutexattr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_mutexattr_getpshared(a1: *mut pthread_mutexattr_t, a2: *mut c_int) -> c_int;
    #[cfg(all(feature = "pthread", feature = "pthread_mutex_types"))]
    pub fn pthread_mutexattr_gettype(a1: *const pthread_mutexattr_t, a2: *mut c_int) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_mutexattr_init(a1: *mut pthread_mutexattr_t) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_mutexattr_setpshared(a1: *mut pthread_mutexattr_t, a2: c_int) -> c_int;
    #[cfg(all(feature = "pthread", feature = "pthread_mutex_types"))]
    pub fn pthread_mutexattr_settype(a1: *mut pthread_mutexattr_t, a2: c_int) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_once(a1: *mut pthread_once_t, a2: unsafe extern "C" fn()) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_setname_np(a1: pthread_t, a2: *const c_char) -> c_int;
    #[cfg(feature = "pthread")]
    pub fn pthread_yield() -> c_void;
    #[cfg(feature = "file_stream")]
    pub fn puts(a1: *const c_char) -> c_int;
    pub fn pwritev(a1: c_int, a2: *const iovec, a3: c_int, a4: off_t) -> ssize_t;
    pub fn qsort(
        a1: *mut c_void,
        a2: size_t,
        a3: size_t,
        a4: unsafe extern "C" fn(a1: *const c_void, a2: *const c_void) -> c_int,
    ) -> c_void;
    pub fn rand() -> c_int;
    pub fn readdir_r(a1: *mut DIR, a2: *mut dirent, a3: *mut *mut dirent) -> c_int;
    pub fn readv(a1: c_int, a2: *const iovec, a3: c_int) -> ssize_t;
    pub fn realloc(a1: *mut c_void, a2: size_t) -> *mut c_void;
    #[cfg(feature = "file_stream")]
    pub fn rewind(a1: *mut FILE) -> c_void;
    pub fn sched_get_priority_max(a1: c_int) -> c_int;
    pub fn sched_get_priority_min(a1: c_int) -> c_int;
    pub fn sem_getvalue(a1: *mut sem_t, a2: *mut c_int) -> c_int;
    pub fn sem_init(a1: *mut sem_t, a2: c_int, a3: c_uint) -> c_int;
    #[cfg(feature = "libc_locale")]
    pub fn setlocale(a1: c_int, a2: *const c_char) -> *mut c_char;
    pub fn setlogmask(a1: c_int) -> c_int;
    #[cfg(feature = "net")]
    pub fn shutdown(a1: c_int, a2: c_int) -> c_int;
    pub fn sigaddset(a1: *mut sigset_t, a2: c_int) -> c_int;
    pub fn sigdelset(a1: *mut sigset_t, a2: c_int) -> c_int;
    pub fn sigemptyset(a1: *mut sigset_t) -> c_int;
    pub fn sigfillset(a1: *mut sigset_t) -> c_int;
    pub fn sigismember(a1: *const sigset_t, a2: c_int) -> c_int;
    pub fn signal(a1: c_int, a2: _sa_handler_t) -> _sa_handler_t;
    pub fn sleep(a1: c_uint) -> c_uint;
    pub fn snprintf(a1: *mut c_char, a2: size_t, a3: *const c_char, a4: ...) -> c_int;
    pub fn sprintf(a1: *mut c_char, a2: *const c_char, a3: ...) -> c_int;
    pub fn sq_addafter(a1: *mut sq_entry_t, a2: *mut sq_entry_t, a3: *mut sq_queue_t) -> c_void;
    pub fn sq_addfirst(a1: *mut sq_entry_t, a2: *mut sq_queue_t) -> c_void;
    pub fn sq_addlast(a1: *mut sq_entry_t, a2: *mut sq_queue_t) -> c_void;
    pub fn sq_rem(a1: *mut sq_entry_t, a2: *mut sq_queue_t) -> c_void;
    pub fn sq_remafter(a1: *mut sq_entry_t, a2: *mut sq_queue_t) -> sq_entry_t;
    pub fn sq_remfirst(a1: *mut sq_queue_t) -> *mut sq_entry_t;
    pub fn sq_remlast(a1: *mut sq_queue_t) -> *mut sq_entry_t;
    pub fn srand(a1: c_uint) -> c_void;
    pub fn sscanf(a1: *const c_char, a2: *const c_char, a3: ...) -> c_int;
    pub fn strcasecmp(a1: *const c_char, a2: *const c_char) -> c_int;
    pub fn strcasestr(a1: *const c_char, a2: *const c_char) -> *mut c_char;
    pub fn strcat(a1: *mut c_char, a2: *const c_char) -> *mut c_char;
    pub fn strchr(a1: *const c_char, a2: c_int) -> c_char;
    pub fn strcmp(a1: *const c_char, a2: *const c_char) -> c_int;
    #[cfg(feature = "libc_locale")]
    pub fn strcoll(a1: *const c_char, a2: *const c_char) -> c_int;
    pub fn strcpy(a1: *mut c_char, a2: *const c_char) -> *mut c_char;
    pub fn strcspn(a1: *const c_char, a2: *const c_char) -> size_t;
    pub fn strdup(a1: *const c_char) -> *mut c_char;
    pub fn strerror(a1: c_int) -> *mut c_char;
    pub fn strerror_r(a1: c_int, a2: *mut c_char, a3: size_t) -> c_int;
    pub fn strftime(a1: *mut c_char, a2: size_t, a3: *const c_char, a4: *const tm) -> size_t;
    pub fn strlen(a1: *const c_char) -> size_t;
    pub fn strncasecmp(a1: *const c_char, a2: *const c_char, a3: size_t) -> c_int;
    pub fn strncat(a1: *mut c_char, a2: *const c_char, a3: size_t) -> *mut c_char;
    pub fn strncmp(a1: *const c_char, a2: *const c_char, a3: size_t) -> c_int;
    pub fn strncpy(a1: *mut c_char, a2: *const c_char, a3: size_t) -> *mut c_char;
    pub fn strndup(a1: *const c_char, a2: size_t) -> *mut c_char;
    pub fn strnlen(a1: *const c_char, a2: size_t) -> size_t;
    pub fn strpbrk(a1: *const c_char, a2: *const c_char) -> *mut c_char;
    pub fn strrchr(a1: *const c_char, a2: c_int) -> *mut c_char;
    pub fn strspn(a1: *const c_char, a2: *const c_char) -> size_t;
    pub fn strstr(a1: *const c_char, a2: *const c_char) -> c_char;
    #[cfg(feature = "have_double")]
    pub fn strtod(a1: *const c_char, a2: *mut *mut c_char) -> f64;
    pub fn strtoimax(a1: *const c_char, a2: *mut *mut c_char, a3: c_int) -> intmax_t;
    pub fn strtok(a1: *mut c_char, a2: *const c_char) -> *mut c_char;
    pub fn strtok_r(a1: *mut c_char, a2: *const c_char, a3: *mut *mut c_char) -> *mut c_char;
    pub fn strtol(a1: *const c_char, a2: *mut *mut c_char, a3: c_int) -> c_long;
    #[cfg(feature = "have_long_long")]
    pub fn strtoll(a1: *const c_char, a2: *mut *mut c_char, a3: c_int) -> c_longlong;
    pub fn strtoul(a1: *const c_char, a2: *mut *mut c_char, a3: c_int) -> c_ulong;
    #[cfg(feature = "have_long_long")]
    pub fn strtoull(a1: *const c_char, a2: *mut *mut c_char, a3: c_int) -> c_ulonglong;
    pub fn strtoumax(a1: *const c_char, a2: *mut *mut c_char, a3: c_int) -> uintmax_t;
    #[cfg(feature = "libc_locale")]
    pub fn strxfrm(a1: *mut c_char, a2: *const c_char, a3: size_t) -> size_t;
    pub fn swab(a1: *const c_void, a2: *mut c_void, a3: ssize_t) -> c_void;
    pub fn swprintf(a1: *mut wchar_t, a2: size_t, a3: *const wchar_t, a4: ...) -> c_int;
    pub fn syslog(a1: c_int, a2: *const c_char, a3: ...) -> c_void;
    #[cfg(feature = "serial_termios")]
    pub fn tcflush(a1: c_int, a2: c_int) -> c_int;
    #[cfg(feature = "serial_termios")]
    pub fn tcgetattr(a1: c_int, a2: *mut termios) -> c_int;
    #[cfg(feature = "serial_termios")]
    pub fn tcsetattr(a1: c_int, a2: c_int, a3: *const termios) -> c_int;
    pub fn telldir(a1: *mut DIR) -> off_t;
    pub fn time(a1: *mut time_t) -> time_t;
    pub fn tolower(a1: c_int) -> c_int;
    pub fn toupper(a1: c_int) -> c_int;
    pub fn towlower(a1: wint_t) -> wint_t;
    pub fn towupper(a1: wint_t) -> wint_t;
    #[cfg(feature = "mountpoint")]
    pub fn truncate(a1: off_t) -> *const c_char;
    #[cfg(not(feature = "have_long_long"))]
    pub fn ub16divub16(a1: uu16, a2: uu16) -> uu16;
    #[cfg(not(feature = "have_long_long"))]
    pub fn ub16mulub16(a1: uu16, a2: uu16) -> uu16;
    #[cfg(not(feature = "have_long_long"))]
    pub fn ub16sqr(a1: uu16) -> uu16;
    pub fn uname(a1: *mut utsname) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn ungetc(a1: c_int, a2: *mut FILE) -> c_int;
    pub fn usleep(a1: useconds_t) -> c_int;
    pub fn vasprintf(a1: *mut *mut c_char, a2: *const c_char, a3: va_list) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn vfprintf(a1: *mut FILE, a2: *const c_char, a3: va_list) -> c_int;
    pub fn vprintf(a1: *const c_char, a2: va_list) -> c_int;
    #[cfg(feature = "file_stream")]
    pub fn vscanf(a1: *const c_char, a2: va_list) -> c_int;
    pub fn vsnprintf(a1: *mut c_char, a2: size_t, a3: *const c_char, a4: va_list) -> c_int;
    pub fn vsprintf(a1: *mut c_char, a2: *const c_char, a3: va_list) -> c_int;
    pub fn vsscanf(a1: *const c_char, a2: *const c_char, a3: va_list) -> c_int;
    pub fn vsyslog(a1: c_int, a2: *const c_char, a3: va_list) -> c_void;
    pub fn wcrtomb(a1: *mut c_char, a2: wchar_t, a3: *mut mbstate_t) -> size_t;
    pub fn wcscmp(a1: *const wchar_t, a2: *const wchar_t) -> c_int;
    pub fn wcscoll(a1: *const wchar_t, a2: *const wchar_t) -> c_int;
    pub fn wcsftime(a1: *mut wchar_t, a2: size_t, a3: *const wchar_t, a4: *const tm) -> size_t;
    pub fn wcslcpy(a1: *mut wchar_t, a2: *const wchar_t, a3: size_t) -> size_t;
    pub fn wcslen(a1: *const wchar_t) -> size_t;
    pub fn wcsnrtombs(
        a1: *mut c_char,
        a2: *const *mut wchar_t,
        a3: size_t,
        a4: size_t,
        a5: *mut mbstate_t,
    ) -> size_t;
    pub fn wcsrtombs(
        a1: *mut c_char,
        a2: *const *mut wchar_t,
        a3: size_t,
        a4: *mut mbstate_t,
    ) -> size_t;
    pub fn wcstod(a1: *const wchar_t, a2: *mut *mut wchar_t) -> f64;
    pub fn wcstof(a1: *const wchar_t, a2: *mut *mut wchar_t) -> f32;
    pub fn wcstol(a1: *const wchar_t, a2: *mut *mut wchar_t, a3: c_int) -> c_long;
    #[cfg(feature = "have_long_double")]
    pub fn wcstold(a1: *const wchar_t, a2: *mut *mut wchar_t) -> c_ldouble;
    #[cfg(feature = "have_long_long")]
    pub fn wcstoll(a1: *const wchar_t, a2: *mut *mut wchar_t, a3: c_int) -> c_longlong;
    pub fn wcstombs(a1: *mut c_char, a2: *const wchar_t, a3: size_t) -> size_t;
    pub fn wcstoul(a1: *const wchar_t, a2: *mut *mut wchar_t, a3: c_int) -> c_ulong;
    pub fn wcsxfrm(a1: *mut wchar_t, a2: *const wchar_t, a3: size_t) -> size_t;
    pub fn wctob(a1: wint_t) -> c_int;
    pub fn wctomb(a1: *mut c_char, a2: wchar_t) -> c_int;
    pub fn wctype(a1: *const c_char) -> wctype_t;
    pub fn wmemchr(a1: *const wchar_t, a2: wchar_t, a3: size_t) -> *mut wchar_t;
    pub fn wmemcmp(a1: *const wchar_t, a2: *const wchar_t, a3: size_t) -> c_int;
    pub fn wmemcpy(a1: *mut wchar_t, a2: *const wchar_t, a3: size_t) -> *mut wchar_t;
    pub fn wmemmove(a1: *mut wchar_t, a2: *const wchar_t, a3: size_t) -> *mut wchar_t;
    pub fn wmemset(a1: *mut wchar_t, a2: wchar_t, a3: size_t) -> *mut wchar_t;
    pub fn writev(a1: c_int, a2: *const iovec, a3: c_int) -> ssize_t;
}

pub const EPERM: c_int = 1;
pub const ENOENT: c_int = 2;
pub const ESRCH: c_int = 3;
pub const EINTR: c_int = 4;
pub const EIO: c_int = 5;
pub const ENXIO: c_int = 6;
pub const E2BIG: c_int = 7;
pub const ENOEXEC: c_int = 8;
pub const EBADF: c_int = 9;
pub const ECHILD: c_int = 10;
pub const EAGAIN: c_int = 11;
pub const EWOULDBLOCK: c_int = EAGAIN;
pub const ENOMEM: c_int = 12;
pub const EACCES: c_int = 13;
pub const EFAULT: c_int = 14;
pub const ENOTBLK: c_int = 15;
pub const EBUSY: c_int = 16;
pub const EEXIST: c_int = 17;
pub const EXDEV: c_int = 18;
pub const ENODEV: c_int = 19;
pub const ENOTDIR: c_int = 20;
pub const EISDIR: c_int = 21;
pub const EINVAL: c_int = 22;
pub const ENFILE: c_int = 23;
pub const EMFILE: c_int = 24;
pub const ENOTTY: c_int = 25;
pub const ETXTBSY: c_int = 26;
pub const EFBIG: c_int = 27;
pub const ENOSPC: c_int = 28;
pub const ESPIPE: c_int = 29;
pub const EROFS: c_int = 30;
pub const EMLINK: c_int = 31;
pub const EPIPE: c_int = 32;
pub const EDOM: c_int = 33;
pub const ERANGE: c_int = 34;
pub const ENOMSG: c_int = 35;
pub const EIDRM: c_int = 36;
pub const ECHRNG: c_int = 37;
pub const EL2NSYNC: c_int = 38;
pub const EL3HLT: c_int = 39;
pub const EL3RST: c_int = 40;
pub const ELNRNG: c_int = 41;
pub const EUNATCH: c_int = 42;
pub const ENOCSI: c_int = 43;
pub const EL2HLT: c_int = 44;
pub const EDEADLK: c_int = 45;
pub const ENOLCK: c_int = 46;

pub const EBADE: c_int = 50;
pub const EBADR: c_int = 51;
pub const EXFULL: c_int = 52;
pub const ENOANO: c_int = 53;
pub const EBADRQC: c_int = 54;
pub const EBADSLT: c_int = 55;
pub const EDEADLOCK: c_int = 56;
pub const EBFONT: c_int = 57;

pub const ENOSTR: c_int = 60;
pub const ENODATA: c_int = 61;
pub const ETIME: c_int = 62;
pub const ENOSR: c_int = 63;
pub const ENONET: c_int = 64;
pub const ENOPKG: c_int = 65;
pub const EREMOTE: c_int = 66;
pub const ENOLINK: c_int = 67;
pub const EADV: c_int = 68;
pub const ESRMNT: c_int = 69;
pub const ECOMM: c_int = 70;
pub const EPROTO: c_int = 71;

pub const EMULTIHOP: c_int = 74;
pub const ELBIN: c_int = 75;
pub const EDOTDOT: c_int = 76;
pub const EBADMSG: c_int = 77;

pub const EFTYPE: c_int = 79;
pub const ENOTUNIQ: c_int = 80;
pub const EBADFD: c_int = 81;
pub const EREMCHG: c_int = 82;
pub const ELIBACC: c_int = 83;
pub const ELIBBAD: c_int = 84;
pub const ELIBSCN: c_int = 85;
pub const ELIBMAX: c_int = 86;
pub const ELIBEXEC: c_int = 87;
pub const ENOSYS: c_int = 88;
pub const ENMFILE: c_int = 89;
pub const ENOTEMPTY: c_int = 90;
pub const ENAMETOOLONG: c_int = 91;
pub const ELOOP: c_int = 92;

pub const EOPNOTSUPP: c_int = 95;
pub const EPFNOSUPPORT: c_int = 96;

pub const ECONNRESET: c_int = 104;
pub const ENOBUFS: c_int = 105;
pub const EAFNOSUPPORT: c_int = 106;
pub const EPROTOTYPE: c_int = 107;
pub const ENOTSOCK: c_int = 108;
pub const ENOPROTOOPT: c_int = 109;
pub const ESHUTDOWN: c_int = 110;
pub const ECONNREFUSED: c_int = 111;
pub const EADDRINUSE: c_int = 112;
pub const ECONNABORTED: c_int = 113;
pub const ENETUNREACH: c_int = 114;
pub const ENETDOWN: c_int = 115;
pub const ETIMEDOUT: c_int = 116;
pub const EHOSTDOWN: c_int = 117;
pub const EHOSTUNREACH: c_int = 118;
pub const EINPROGRESS: c_int = 119;
pub const EALREADY: c_int = 120;
pub const EDESTADDRREQ: c_int = 121;
pub const EMSGSIZE: c_int = 122;
pub const EPROTONOSUPPORT: c_int = 123;
pub const ESOCKTNOSUPPORT: c_int = 124;
pub const EADDRNOTAVAIL: c_int = 125;
pub const ENETRESET: c_int = 126;
pub const EISCONN: c_int = 127;
pub const ENOTCONN: c_int = 128;
pub const ETOOMANYREFS: c_int = 129;
pub const EPROCLIM: c_int = 130;
pub const EUSERS: c_int = 131;
pub const EDQUOT: c_int = 132;
pub const ESTALE: c_int = 133;
pub const ENOTSUP: c_int = 134;
pub const ENOMEDIUM: c_int = 135;
pub const ENOSHARE: c_int = 136;
pub const ECASECLASH: c_int = 137;
pub const EILSEQ: c_int = 138;
pub const EOVERFLOW: c_int = 139;
pub const ECANCELED: c_int = 140;
pub const ENOTRECOVERABLE: c_int = 141;
pub const EOWNERDEAD: c_int = 142;
pub const ESTRPIPE: c_int = 143;

pub const CLOCK_MONOTONIC: c_int = 1;
pub const CLOCK_REALTIME: c_int = 0;
pub const EXIT_FAILURE: c_int = 1;
pub const EXIT_SUCCESS: c_int = 0;
pub const FD_CLOEXEC: c_int = 1;
pub const F_GETFD: c_int = 1;
pub const F_GETFL: c_int = 2;
pub const F_SETFD: c_int = 8;
pub const F_SETFL: c_int = 9;
pub const O_ACCMODE: c_int = O_RDWR;
pub const O_APPEND: c_int = 1 << 4;
pub const O_CLOEXEC: c_int = 1 << 10;
pub const O_CREAT: c_int = 1 << 2;
pub const O_DIRECT: c_int = 1 << 9;
pub const O_DIRECTORY: c_int = 1 << 11;
pub const O_DSYNC: c_int = O_SYNC;
pub const O_EXCL: c_int = 1 << 3;
pub const O_NOCTTY: c_int = 0;
pub const O_NONBLOCK: c_int = 1 << 6;
pub const O_RDONLY: c_int = 1 << 0;
pub const O_RDOK: c_int = O_RDONLY;
pub const O_RDWR: c_int = O_RDOK | O_WROK;
pub const O_SYNC: c_int = 1 << 7;
pub const O_TRUNC: c_int = 1 << 5;
pub const O_WRONLY: c_int = 1 << 1;
pub const O_WROK: c_int = O_WRONLY;
pub const PIPE_BUF: usize = _POSIX_PIPE_BUF;
pub const _POSIX_PIPE_BUF: usize = 512;
pub const POLLERR: i16 = 0x08;
pub const POLLHUP: i16 = 0x10;
pub const POLLIN: i16 = 0x01;
pub const POLLNVAL: i16 = 0x20;
pub const POLLOUT: i16 = 0x04;
pub const POLLPRI: i16 = 0x02;
pub const POLLRDBAND: i16 = 0x01;
pub const POLLRDNORM: i16 = 0x01;
pub const POLLWRBAND: i16 = 0x04;
pub const POLLWRNORM: i16 = 0x04;
pub const PRIO_PGRP: c_int = 2;
pub const PRIO_PROCESS: c_int = 1;
pub const PRIO_USER: c_int = 3;
pub const RLIMIT_AS: c_int = 7;
pub const RLIMIT_CORE: c_int = 1;
pub const RLIMIT_CPU: c_int = 2;
pub const RLIMIT_DATA: c_int = 3;
pub const RLIMIT_FSIZE: c_int = 4;
pub const RLIMIT_NOFILE: c_int = 5;
pub const RLIMIT_STACK: c_int = 6;
pub const SEEK_CUR: c_int = 1;
pub const SEEK_END: c_int = 2;
pub const SEEK_SET: c_int = 0;
pub const S_IFBLK: c_uint = 6 << 12;
pub const S_IFCHR: c_uint = 2 << 12;
pub const S_IFDIR: c_uint = 4 << 12;
pub const S_IFIFO: c_uint = 1 << 12;
pub const S_IFLNK: c_uint = 10 << 12;
pub const S_IFMTD: c_uint = 9 << 12;
pub const S_IFMT: c_uint = 15 << 12;
pub const S_IFREG: c_uint = 8 << 12;
pub const S_IFSOCK: c_uint = 12 << 12;
pub const S_IRGRP: c_uint = 1 << 5;
pub const S_IROTH: c_uint = 1 << 2;
pub const S_IRUSR: c_uint = 1 << 8;
pub const S_IRWXG: c_uint = 7 << 3;
pub const S_IRWXO: c_uint = 7 << 0;
pub const S_IRWXU: c_uint = 7 << 6;
pub const S_ISGID: c_uint = 1 << 10;
pub const S_ISUID: c_uint = 1 << 11;
pub const S_ISVTX: c_uint = 1 << 9;
pub const S_IWGRP: c_uint = 1 << 4;
pub const S_IWOTH: c_uint = 1 << 1;
pub const S_IWUSR: c_uint = 1 << 7;
pub const S_IXGRP: c_uint = 1 << 3;
pub const S_IXOTH: c_uint = 1 << 0;
pub const S_IXUSR: c_uint = 1 << 6;
pub const STDERR_FILENO: c_int = 2;
pub const STDIN_FILENO: c_int = 0;
pub const STDOUT_FILENO: c_int = 1;
pub const F_OK: c_int = 0;
pub const X_OK: c_int = 1;
pub const W_OK: c_int = 2;
pub const R_OK: c_int = 4;

pub const DT_UNKNOWN: u8 = 0;
pub const DT_FIFO: u8 = 1;
pub const DT_CHR: u8 = 2;
pub const DT_SEM: u8 = 3;
pub const DT_DIR: u8 = 4;
pub const DT_MQ: u8 = 5;
pub const DT_BLK: u8 = 6;
pub const DT_SHM: u8 = 7;
pub const DT_REG: u8 = 8;
pub const DT_MTD: u8 = 9;
pub const DT_LNK: u8 = 10;
pub const DT_SOCK: u8 = 12;
