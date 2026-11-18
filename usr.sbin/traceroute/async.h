/*
 * Optional bounded asynchronous collection.  Keep the existing packet parser
 * and printer: replies are saved here and replayed in TTL/probe order.
 */
#include <stdint.h>
#include <time.h>

#define	ASYNC_DEFAULT_TTL	64
#define	ASYNC_TTL_STEP		5

enum async_match {
	ASYNC_NO_MATCH,
	ASYNC_HOP,
	ASYNC_UNREACHABLE,
	ASYNC_DESTINATION,
};

struct async_probe {
	bool sent, done, reply;
	int64_t deadline;
	struct timeval t1, t2;
	struct async_received received;
};

static struct async_probe *async_probes;
static int async_count;

/* Store monotonic time in timeval so the existing RTT formatter can use it. */
static int64_t
async_now(struct timeval *tv)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1)
		err(1, "clock_gettime");
	if (tv != NULL) {
		tv->tv_sec = ts.tv_sec;
		tv->tv_usec = ts.tv_nsec / 1000;
	}
	return ((int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
}

#include "async_lookup.h"

static int	async_receive(struct async_received *);
static enum async_match async_match_reply(struct async_received *, int);
static void	async_send(int, int, struct timeval *);
#include "async_display.h"

/*
 * Keep each slot representable in its wire sequence number or port.
 * The initial range is validated by the caller.
 */
static int
async_ttl_limit(int first, int count, int available)
{

	return (MIN(UINT8_MAX, first + available / count - 1));
}

static int
async_trace(int s, int first, int last, int count, int available,
    int unreachable_count, unsigned int pause, bool extend)
{
	int *pending, npending, slot, i, received, cc;
	int completed, end, last_ttl, nfds;
	int last_sent = first, sent = 0, replies = 0;
	int stage_pending, stage_probe, stage_start, next_end;
	int64_t now, wake, nextsend[UINT8_MAX + 1] = { 0 };
	int next_probe[UINT8_MAX + 1] = { 0 };
	int unreachable[UINT8_MAX + 1] = { 0 };
	int selected_probe, probe, row, ttl;
	enum async_match match;
	struct async_probe *p;
	struct async_received reply = { 0 };
	fd_set readfds, writefds;
	struct timeval timeout;
	struct timeval received_at;

	if (s >= FD_SETSIZE)
		errx(1, "asynchronous receive socket exceeds FD_SETSIZE");
	end = (last - first + 1) * count;
	async_count = extend ?
	    (async_ttl_limit(first, count, available) - first + 1) * count :
	    end;
	async_probes = calloc(async_count, sizeof(*async_probes));
	pending = calloc(async_count, sizeof(*pending));
	if (async_probes == NULL || pending == NULL)
		err(1, "calloc asynchronous probes");

	npending = completed = 0;
	stage_pending = end / count;
	stage_start = stage_probe = 0;
	/* Extended TTLs are outside the first pass. */
	for (slot = end; slot < async_count; slot += count) {
		async_probes[slot].done = true;
		next_probe[first + slot / count] = 1;
	}
	async_display_begin();
	for (;;) {
		async_display_check_signal();
		/* Bound the receive batch so unrelated traffic cannot starve sends. */
		for (received = 0; received < 256; received++) {
			cc = async_receive(&reply);
			if (cc == -1) {
				if (errno == EINTR)
					continue;
				if (errno == EAGAIN || errno == EWOULDBLOCK)
					break;
				err(1, "receive asynchronous probe");
			}
			now = async_now(&received_at);
			for (i = 0; i < npending; i++) {
				slot = pending[i];
				p = &async_probes[slot];
				if (slot >= end || p->deadline <= now)
					continue;
				match = async_match_reply(&reply, slot + 1);
				if (match == ASYNC_NO_MATCH)
					continue;
				p->done = p->reply = true;
				replies++;
				if (extend && slot >= stage_start &&
				    slot % count == stage_probe)
					stage_pending--;
				row = slot / count;
				if (match == ASYNC_DESTINATION ||
				    (match == ASYNC_UNREACHABLE &&
				    ++unreachable[row] >= unreachable_count)) {
					end = (row + 1) * count;
					extend = false;
				}
				p->received = reply;
				p->t2 = received_at;
				lookup_enqueue(reply.from);
				pending[i] = pending[--npending];
				break;
			}
		}

		now = async_now(NULL);
		wake = INT64_MAX;
		for (i = 0; i < npending;) {
			slot = pending[i];
			p = &async_probes[slot];
			if (slot >= end || p->deadline <= now) {
				if (extend && slot >= stage_start &&
				    slot % count == stage_probe)
					stage_pending--;
				p->done = true;
				pending[i] = pending[--npending];
				continue;
			}
			if (p->deadline < wake)
				wake = p->deadline;
			i++;
		}
		if (extend && stage_pending == 0) {
			if (end == async_count)
				extend = false;
			else {
				stage_start = end;
				stage_probe = 1;
				next_end = MIN(end + ASYNC_TTL_STEP * count,
				    async_count);
				stage_pending = (next_end - end) / count;
				end = next_end;
			}
		}
		last_ttl = first + end / count - 1;
		async_display_update(now, sent, npending, replies, first,
		    MIN(last_sent, last_ttl), count);
		/* Finish once every row through the destination is settled. */
		while (completed < end && async_probes[completed].done)
			completed++;
		if (completed >= end)
			break;
		nfds = s + 1;
		FD_ZERO(&readfds);
		FD_ZERO(&writefds);
		FD_SET(s, &readfds);
		lookup_selectfds(&readfds, &writefds, &nfds);
		/*
		 * Among ready hops, send the lowest probe index first,
		 * breaking ties by TTL.  Each hop has its own send timer.
		 */
		slot = -1;
		selected_probe = count;
		for (ttl = first; ttl <= last_ttl; ttl++) {
			probe = next_probe[ttl];
			if (probe == count)
				continue;
			if (nextsend[ttl] > now) {
				if (nextsend[ttl] < wake)
					wake = nextsend[ttl];
				continue;
			}
			if (probe < selected_probe) {
				slot = (ttl - first) * count + probe;
				selected_probe = probe;
			}
		}
		if (slot >= 0) {
			ttl = first + slot / count;
			next_probe[ttl]++;
			last_sent = MAX(last_sent, ttl);
			sent++;
			p = &async_probes[slot];
			p->sent = true;
			now = async_now(&p->t1);
			p->deadline = now + (int64_t)waittime * 1000000;
			pending[npending++] = slot;
			async_send(slot + 1, ttl, &p->t1);
			nextsend[ttl] = now + (int64_t)pause * 1000;
			continue;
		}
		now = async_now(NULL);
		wake = wake > now ? wake - now : 0;
		if (async_screen_active)
			wake = MIN(wake, 100000);
		timeout.tv_sec = wake / 1000000;
		timeout.tv_usec = wake % 1000000;
		if (select(nfds, &readfds, &writefds, NULL, &timeout) == -1) {
			if (errno == EINTR)
				continue;
			err(1, "select asynchronous probes");
		}
	}
	async_display_check_signal();
	async_display_finish();
	free(pending);
	return (first + end / count - 1);
}

static void
async_free(void)
{
	free(async_probes);
	async_probes = NULL;
	async_count = 0;
}
