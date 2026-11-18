/*
 * Numeric progress using terminal capabilities.  The preview does not
 * depend on name or AS lookups.
 */
#include <sys/ioctl.h>

#include <signal.h>
#include <termcap.h>

static char *async_screen_enter, *async_screen_leave, *async_screen_clear;
static char *async_screen_move, *async_screen_eol, *async_screen_up;
static char *async_cursor_hide, *async_cursor_show;
static int async_screen_active, async_screen_ready, async_screen_alternate;
static int async_screen_rows, async_screen_columns, async_screen_height;
static int64_t async_screen_updated;
static volatile sig_atomic_t async_screen_signal;
static struct sigaction async_screen_oldint, async_screen_oldterm;
static int async_screen_intset, async_screen_termset;

static void
async_display_finish(void)
{
	struct winsize size;
	int i;

	if (async_screen_active) {
		if (async_cursor_show != NULL)
			tputs(async_cursor_show, 1, putchar);
		if (async_screen_alternate)
			tputs(async_screen_leave, 1, putchar);
		else if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0 &&
		    size.ws_row == async_screen_rows &&
		    size.ws_col == async_screen_columns) {
			for (i = 0; i < async_screen_height; i++)
				tputs(async_screen_up, 1, putchar);
			for (i = 0; i < async_screen_height; i++) {
				putchar('\r');
				tputs(async_screen_eol, 1, putchar);
				fputs("\r\n", stdout);
			}
			for (i = 0; i < async_screen_height; i++)
				tputs(async_screen_up, 1, putchar);
			putchar('\r');
		} else
			fputs("\r\n", stdout);
		(void)fflush(stdout);
		async_screen_active = 0;
	}
	if (async_screen_intset) {
		(void)sigaction(SIGINT, &async_screen_oldint, NULL);
		async_screen_intset = 0;
	}
	if (async_screen_termset) {
		(void)sigaction(SIGTERM, &async_screen_oldterm, NULL);
		async_screen_termset = 0;
	}
}

/* Load the terminal database before entering capability mode. */
static void
async_display_init(int capacity_ttl)
{
	const char *term;
	struct winsize size;

	if (!isatty(STDOUT_FILENO) || (term = getenv("TERM")) == NULL ||
	    strcmp(term, "dumb") == 0 ||
	    ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == -1 ||
	    size.ws_row < 6 || size.ws_col < 20 || tgetent(NULL, term) != 1)
		return;
	async_screen_enter = tgetstr("ti", NULL);
	async_screen_leave = tgetstr("te", NULL);
	async_screen_clear = tgetstr("cl", NULL);
	async_screen_move = tgetstr("cm", NULL);
	async_screen_eol = tgetstr("ce", NULL);
	async_screen_up = tgetstr("up", NULL);
	async_cursor_hide = tgetstr("vi", NULL);
	async_cursor_show = tgetstr("ve", NULL);
	async_screen_alternate = async_screen_enter != NULL &&
	    async_screen_leave != NULL && async_screen_clear != NULL &&
	    async_screen_move != NULL;
	if (async_screen_eol == NULL ||
	    (!async_screen_alternate && async_screen_up == NULL))
		return;
	if (async_cursor_show == NULL)
		async_cursor_hide = NULL;
	async_screen_rows = size.ws_row;
	async_screen_columns = size.ws_col;
	async_screen_height = capacity_ttl - first_ttl + 4;
	if (async_screen_height >= size.ws_row)
		async_screen_height = size.ws_row - 1;
	if (atexit(async_display_finish) == 0)
		async_screen_ready = 1;
}

static void
async_display_signalled(int signo)
{

	async_screen_signal = signo;
}

static void
async_display_begin(void)
{
	struct sigaction action;
	int i;

	if (!async_screen_ready)
		return;
	memset(&action, 0, sizeof(action));
	action.sa_handler = async_display_signalled;
	sigemptyset(&action.sa_mask);
	if (sigaction(SIGINT, NULL, &async_screen_oldint) == -1)
		return;
	if (async_screen_oldint.sa_handler != SIG_IGN) {
		if (sigaction(SIGINT, &action, NULL) == -1)
			return;
		async_screen_intset = 1;
	}
	if (sigaction(SIGTERM, &action, &async_screen_oldterm) == -1) {
		async_display_finish();
		return;
	}
	async_screen_termset = 1;
	async_screen_signal = 0;
	async_screen_updated = 0;
	async_screen_active = 1;
	if (async_screen_alternate) {
		tputs(async_screen_enter, 1, putchar);
		tputs(async_screen_clear, 1, putchar);
	} else {
		for (i = 0; i < async_screen_height; i++)
			fputs("\r\n", stdout);
	}
	if (async_cursor_hide != NULL)
		tputs(async_cursor_hide, 1, putchar);
	(void)fflush(stdout);
}

static void
async_display_check_signal(void)
{
	int signo;

	if ((signo = async_screen_signal) == 0)
		return;
	async_display_finish();
	(void)raise(signo);
	_exit(128 + signo);
}

/* Every line is clipped before the last column to prevent line wrapping. */
static void
async_display_line(int row, char *line, int columns)
{

	if (strlen(line) >= (size_t)columns)
		line[columns - 1] = '\0';
	if (async_screen_alternate)
		tputs(tgoto(async_screen_move, 0, row), 1, putchar);
	else
		putchar('\r');
	tputs(async_screen_eol, 1, putchar);
	fputs(line, stdout);
	if (!async_screen_alternate)
		fputs("\r\n", stdout);
}

static void
async_display_update(int64_t now, int sent, int pending, int replies,
    int last)
{
	struct winsize size;
	struct async_probe *p;
	struct sockaddr_in *to;
	char line[1024], part[80], address[INET_ADDRSTRLEN];
	int row, ttl, first, probe, slot, columns, rows, i;
	in_addr_t previous;

	if (!async_screen_active || now - async_screen_updated < 100000)
		return;
	async_screen_updated = now;
	if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == -1 ||
	    size.ws_row < 6 || size.ws_col < 20) {
		async_display_finish();
		return;
	}
	/* Reflow can move a relative preview's origin.  Keep the final output
	 * intact by ending that preview instead of guessing its new position. */
	if (!async_screen_alternate &&
	    (size.ws_row != async_screen_rows ||
	    size.ws_col != async_screen_columns)) {
		async_display_finish();
		return;
	}
	columns = size.ws_col;
	if (columns > (int)sizeof(line))
		columns = sizeof(line);
	rows = async_screen_alternate ? size.ws_row : async_screen_height;
	if (!async_screen_alternate) {
		for (i = 0; i < async_screen_height; i++)
			tputs(async_screen_up, 1, putchar);
	}
	to = (struct sockaddr_in *)&whereto;
	(void)inet_ntop(AF_INET, &to->sin_addr, address, sizeof(address));
	(void)snprintf(line, sizeof(line), "traceroute to %s (-Q %d)",
	    address, async_mode);
	async_display_line(0, line, columns);
	(void)snprintf(line, sizeof(line),
	    "Sent %d  Received %d  Pending %d", sent, replies, pending);
	async_display_line(1, line, columns);
	first = last - (rows - 3) + 1;
	if (first < first_ttl)
		first = first_ttl;
	for (row = 2, ttl = first; row < rows - 1; row++, ttl++) {
		line[0] = '\0';
		if (ttl <= last) {
			(void)snprintf(line, sizeof(line), "%2d ", ttl);
			previous = INADDR_ANY;
			for (probe = 0; probe < nprobes; probe++) {
				slot = (ttl - first_ttl) * nprobes + probe;
				p = &async_probes[slot];
				if (p->reply) {
					if (previous != p->from.sin_addr.s_addr) {
						(void)inet_ntop(AF_INET,
						    &p->from.sin_addr, address,
						    sizeof(address));
						(void)snprintf(part, sizeof(part),
						    " %s", address);
						strlcat(line, part, sizeof(line));
						previous = p->from.sin_addr.s_addr;
					}
					(void)snprintf(part, sizeof(part),
					    " %.3f ms", deltaT(&p->t1, &p->t2));
				} else if (!p->sent)
					strlcpy(part, " .", sizeof(part));
				else if (!p->done)
					strlcpy(part, " ?", sizeof(part));
				else
					strlcpy(part, " *", sizeof(part));
				strlcat(line, part, sizeof(line));
				if (strlen(line) >= (size_t)columns - 1)
					break;
			}
		}
		async_display_line(row, line, columns);
	}
	strlcpy(line, ". not sent   ? waiting   * timed out", sizeof(line));
	async_display_line(rows - 1, line, columns);
	(void)fflush(stdout);
}
