/*
 * Keep the blocking libc/Casper and RADB interfaces off the probe loop.
 * Each worker owns its connection and handles one address at a time.
 */
#include <sys/procdesc.h>

#include <signal.h>

enum lookup_kind { LOOKUP_DNS, LOOKUP_AS, LOOKUP_COUNT };

struct lookup_result {
	char name[MAXHOSTNAMELEN + 1];
	unsigned int as;
	bool found;
};

struct lookup_entry {
	struct lookup_entry *hash_next, *next[LOOKUP_COUNT];
	async_addr_t addr;
	struct lookup_result result[LOOKUP_COUNT];
};

struct lookup_worker {
	int fd, procfd;
	struct lookup_entry *head, *tail, *active;
};

static struct lookup_worker lookup_workers[LOOKUP_COUNT] = {
	{ .fd = -1, .procfd = -1 },
	{ .fd = -1, .procfd = -1 }
};
static struct lookup_entry *lookup_cache[256];

static void
lookup_close(struct lookup_worker *worker)
{

	if (worker->fd != -1) {
		close(worker->fd);
		close(worker->procfd);
		worker->fd = worker->procfd = -1;
	}
}

static void
lookup_cleanup(void)
{
	struct lookup_entry *entry;
	int i;

	for (i = 0; i < LOOKUP_COUNT; i++)
		lookup_close(&lookup_workers[i]);
	for (i = 0; i < (int)nitems(lookup_cache); i++) {
		while ((entry = lookup_cache[i]) != NULL) {
			lookup_cache[i] = entry->hash_next;
			free(entry);
		}
	}
}

static void
lookup_child(int fd, enum lookup_kind kind, bool sandbox)
{
	struct lookup_result result;
	async_addr_t addr;
	char numeric[INET6_ADDRSTRLEN];
	void *connection = NULL;
	int status;

	close(async_receive_fd());
	close(sndsock);
	if (kind == LOOKUP_DNS && sandbox && cap_enter() == -1 &&
	    errno != ENOSYS)
		_exit(1);
	if (kind == LOOKUP_AS) {
#ifdef WITH_CASPER
		cap_close(capdns);
#endif
		/* A dead WHOIS peer must not take the traceroute process down. */
		(void)signal(SIGPIPE, SIG_IGN);
	}
	while (recv(fd, &addr, sizeof(addr), 0) == sizeof(addr)) {
		memset(&result, 0, sizeof(result));
		if (kind == LOOKUP_DNS) {
			strlcpy(result.name, async_name(&addr), sizeof(result.name));
			result.found = result.name[0] != '\0';
		} else {
			if (connection == NULL) {
				connection = as_setup(as_server);
				if (connection == NULL)
					break;
				if (cap_enter() == -1 && errno != ENOSYS)
					break;
			}
			async_numeric(&addr, numeric, sizeof(numeric));
			result.as = as_lookup(connection, numeric, ASYNC_FAMILY,
			    &status);
			if (status != 0)
				break;
			result.found = true;
		}
		if (send(fd, &result, sizeof(result), MSG_NOSIGNAL) !=
		    sizeof(result))
			break;
	}
	_exit(0);
}

static void
lookup_start(bool sandbox)
{
	struct lookup_worker *worker;
	int fd[2], i;

	if ((nflag && !as_path) || atexit(lookup_cleanup) != 0)
		return;
	for (i = 0; i < LOOKUP_COUNT; i++) {
		if ((i == LOOKUP_DNS && nflag) || (i == LOOKUP_AS && !as_path))
			continue;
		if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, fd) == -1)
			continue;
		if (fd[0] >= FD_SETSIZE) {
			close(fd[0]);
			close(fd[1]);
			continue;
		}
		worker = &lookup_workers[i];
		switch (pdfork(&worker->procfd, 0)) {
		case -1:
			close(fd[0]);
			close(fd[1]);
			continue;
		case 0:
			close(fd[0]);
			if (i != 0 && lookup_workers[0].fd != -1) {
				close(lookup_workers[0].fd);
				close(lookup_workers[0].procfd);
			}
			lookup_child(fd[1], i, sandbox);
			_exit(0);
		default:
			close(fd[1]);
			worker->fd = fd[0];
			break;
		}
	}
}

static struct lookup_entry **
lookup_slot(async_addr_t addr)
{
	struct lookup_entry **slot;
	const unsigned char *bytes;
	uint32_t key;
	size_t i;

	bytes = (const unsigned char *)&addr;
	key = 0;
	for (i = 0; i < sizeof(addr); i++)
		key = key * 33 + bytes[i];
	slot = &lookup_cache[key % nitems(lookup_cache)];
	while (*slot != NULL && !async_addr_equal(&(*slot)->addr, &addr))
		slot = &(*slot)->hash_next;
	return (slot);
}

static void
lookup_enqueue(async_addr_t addr)
{
	struct lookup_worker *worker;
	struct lookup_entry **slot, *entry;
	int i;

	if (lookup_workers[LOOKUP_DNS].fd == -1 &&
	    lookup_workers[LOOKUP_AS].fd == -1)
		return;
	slot = lookup_slot(addr);
	if (*slot != NULL)
		return;
	if ((entry = calloc(1, sizeof(*entry))) == NULL)
		return;
	entry->addr = addr;
	*slot = entry;
	for (i = 0; i < LOOKUP_COUNT; i++) {
		worker = &lookup_workers[i];
		if (worker->fd == -1)
			continue;
		if (worker->tail != NULL)
			worker->tail->next[i] = entry;
		else
			worker->head = entry;
		worker->tail = entry;
	}
}

static void
lookup_selectfds(fd_set *readfds, fd_set *writefds, int *nfds)
{
	struct lookup_worker *worker;
	struct lookup_result result;
	ssize_t n;
	int i;

	for (i = 0; i < LOOKUP_COUNT; i++) {
		worker = &lookup_workers[i];
		if (worker->fd != -1 && worker->active != NULL) {
			n = recv(worker->fd, &result, sizeof(result), MSG_DONTWAIT);
			if (n == sizeof(result)) {
				worker->active->result[i] = result;
				worker->active = NULL;
			} else if (n != -1 || (errno != EAGAIN && errno != EINTR))
				lookup_close(worker);
		}
		if (worker->fd != -1 && worker->active == NULL &&
		    worker->head != NULL) {
			n = send(worker->fd, &worker->head->addr,
			    sizeof(worker->head->addr), MSG_DONTWAIT | MSG_NOSIGNAL);
			if (n == sizeof(worker->head->addr)) {
				worker->active = worker->head;
				worker->head = worker->head->next[i];
				if (worker->head == NULL)
					worker->tail = NULL;
			} else if (n != -1 || (errno != EAGAIN && errno != EINTR))
				lookup_close(worker);
		}
		if (worker->fd == -1)
			continue;
		if (worker->active != NULL)
			FD_SET(worker->fd, readfds);
		else if (worker->head != NULL)
			FD_SET(worker->fd, writefds);
		else
			continue;
		if (worker->fd >= *nfds)
			*nfds = worker->fd + 1;
	}
}

/* Use results that arrive within a single post-probe lookup budget. */
static void
lookup_finish(void)
{
	fd_set readfds, writefds;
	struct timeval timeout;
	int64_t deadline, remaining;
	int i, nfds;

	deadline = async_now(NULL) + 5000000;
	for (;;) {
		nfds = 0;
		FD_ZERO(&readfds);
		FD_ZERO(&writefds);
		lookup_selectfds(&readfds, &writefds, &nfds);
		remaining = deadline - async_now(NULL);
		if (nfds == 0 || remaining <= 0)
			break;
		timeout.tv_sec = remaining / 1000000;
		timeout.tv_usec = remaining % 1000000;
		if (select(nfds, &readfds, &writefds, NULL, &timeout) < 0 &&
		    errno != EINTR)
			break;
	}
	for (i = 0; i < LOOKUP_COUNT; i++)
		lookup_close(&lookup_workers[i]);
}

static void
lookup_print(async_addr_t addr, const char *numeric)
{
	const struct lookup_entry *entry;

	entry = *lookup_slot(addr);
	if (entry != NULL && entry->result[LOOKUP_AS].found)
		printf(" [AS%u]", entry->result[LOOKUP_AS].as);
	if (entry != NULL && entry->result[LOOKUP_DNS].found &&
	    strcmp(entry->result[LOOKUP_DNS].name, numeric) != 0)
		printf(" %s (%s)", entry->result[LOOKUP_DNS].name, numeric);
	else
		printf(" %s", numeric);
}
