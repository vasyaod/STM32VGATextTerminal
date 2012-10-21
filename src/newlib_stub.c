/**
 * STM32VGATextTerminal
 * 
 * Link: https://github.com/vasyaod/STM32VGATextTerminal
 * Author: vasyaod (vasyaod@mail.ru)
 */

#include <stdlib.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>


caddr_t _sbrk_r(struct _reent *ptr, int incr )
{
	extern int _end;
	static unsigned char *heap = NULL;
	unsigned char *prev_heap;

	if (heap == NULL) {
		heap = (unsigned char *)&_end;
	}
	prev_heap = heap;
	/* check removed to show basic approach */
	heap += incr;

	return (caddr_t) prev_heap;
}
/*
void _exit_r(void *r, int status) {
	while (1) {
	;
	}
}

int _execve_r(void *r, char *name, char **argv, char **env) {
	return -1;
}

int _fork_r(void *r) {
	errno = EAGAIN;
	return -1;
}

int _getpid_r(void *r) {
	return 1;
}

int _kill_r(void*r, int pid, int sig) {
	return (-1);
}

int _link_r(void*r, char *old, char *new) {
	return -1;
}

int _isatty_r(void *r, int file)
{
	return 1;
}

int _lseek_r(void *r, int file, int ptr, int dir)
{
	return 0;
}

int _open_r(void *r, const char *name, int flags, int mode)
{
	return -1;
}

int _close_r(void *r, int file)
{
	return -1;
}

int _fstat_r(void *r,int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _read_r(void *r, int file, char *ptr, int len)
{
	return 0;
}


int _write_r(void *r, int file, char *ptr, int len)
{
	int n;
	switch (file) {
		case STDOUT_FILENO:
			for (n = 0; n < len; n++) {
				TerminalPutchar(*ptr++);
			}
			break;
		default:
			errno = EBADF;
			return -1;
	}

	return len;
}

clock_t _times_r(void *r, struct tms *buf) {
	return -1;
}

int _unlink_r(void *r, char *name) {
	return -1;
}

int _wait_r(void *r,int *status) {
	return -1;
}
*/
