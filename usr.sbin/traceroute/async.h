/*
 * Optional bounded asynchronous collection.  Keep the existing packet parser
 * and printer: replies are saved here and replayed in TTL/probe order.
 */
#include <stdint.h>
#include <time.h>

struct async_probe {
	bool done, reply;
	int cc;
	int64_t deadline;
	struct timeval t1, t2;
	struct sockaddr_in from;
	u_char packet[sizeof(packet)];
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

static void
async_trace(void)
{
	int *pending, npending, slot, i, received, cc, code;
	int completed, end, last_ttl, nfds;
	int64_t now, wake, nextsend[256] = { 0 };
	int next_probe[256] = { 0 }, selected_probe, probe, ttl;
	struct async_probe *p;
	struct outdata data;
	fd_set readfds, writefds;
	struct timeval timeout;
	struct sockaddr_in from;
	struct timeval received_at;
	socklen_t fromlen;
	u_char buf[sizeof(packet)];

	if (s >= FD_SETSIZE)
		errx(1, "asynchronous receive socket exceeds FD_SETSIZE");
	async_count = (max_ttl - first_ttl + 1) * nprobes;
	async_probes = calloc(async_count, sizeof(*async_probes));
	pending = calloc(async_count, sizeof(*pending));
	if (async_probes == NULL || pending == NULL)
		err(1, "calloc asynchronous probes");

	npending = completed = 0;
	end = async_count;
	for (;;) {
		/* Bound the receive batch so unrelated traffic cannot starve sends. */
		for (received = 0; received < 256; received++) {
			fromlen = sizeof(from);
			cc = recvfrom(s, buf, sizeof(buf), MSG_DONTWAIT,
			    (struct sockaddr *)&from, &fromlen);
			if (cc == -1) {
				if (errno == EINTR)
					continue;
				if (errno == EAGAIN || errno == EWOULDBLOCK)
					break;
				err(1, "recvfrom asynchronous probe");
			}
			now = async_now(&received_at);
			for (i = 0; i < npending; i++) {
				slot = pending[i];
				p = &async_probes[slot];
				if (slot >= end || p->deadline <= now)
					continue;
				code = packet_ok(buf, cc, &from, slot + 1);
				if (code == 0)
					continue;
				p->done = p->reply = true;
				if ((code == -2 || code == ICMP_UNREACH_PORT + 1 ||
				    code == ICMP_UNREACH_PROTOCOL + 1) && slot < end)
					end = (slot / nprobes + 1) * nprobes;
				p->cc = cc;
				p->from = from;
				p->t2 = received_at;
				if (slot < end)
					lookup_enqueue(from.sin_addr);
				memcpy(p->packet, buf, cc);
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
				p->done = true;
				pending[i] = pending[--npending];
				continue;
			}
			if (p->deadline < wake)
				wake = p->deadline;
			i++;
		}
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
		selected_probe = nprobes;
		last_ttl = first_ttl + end / nprobes - 1;
		for (ttl = first_ttl; ttl <= last_ttl; ttl++) {
			probe = next_probe[ttl];
			if (probe == nprobes)
				continue;
			if (nextsend[ttl] > now) {
				if (nextsend[ttl] < wake)
					wake = nextsend[ttl];
				continue;
			}
			if (probe < selected_probe) {
				slot = (ttl - first_ttl) * nprobes + probe;
				selected_probe = probe;
			}
		}
		if (slot >= 0) {
			ttl = first_ttl + slot / nprobes;
			next_probe[ttl]++;
			p = &async_probes[slot];
			data.seq = slot + 1;
			data.ttl = ttl;
			now = async_now(&p->t1);
			data.tv = p->t1;
			p->deadline = now + (int64_t)waittime * 1000000;
			pending[npending++] = slot;
			(*proto->prepare)(&data);
			send_probe(slot + 1, data.ttl);
			nextsend[ttl] = now + (int64_t)pausemsecs * 1000;
			continue;
		}
		now = async_now(NULL);
		wake = wake > now ? wake - now : 0;
		timeout.tv_sec = wake / 1000000;
		timeout.tv_usec = wake % 1000000;
		if (select(nfds, &readfds, &writefds, NULL, &timeout) == -1) {
			if (errno == EINTR)
				continue;
			err(1, "select asynchronous probes");
		}
	}
	free(pending);
}

/* A saved reply is consumed once, matching wait_for_reply's interface. */
static int
async_reply(int seq, struct sockaddr_in *from, struct timeval *t1,
    struct timeval *t2)
{
	struct async_probe *p;

	if (seq < 1 || seq > async_count)
		return (0);
	p = &async_probes[seq - 1];
	if (!p->reply)
		return (0);
	p->reply = false;
	*from = p->from;
	*t1 = p->t1;
	*t2 = p->t2;
	memcpy(packet, p->packet, p->cc);
	return (p->cc);
}

static void
async_free(void)
{
	free(async_probes);
	async_probes = NULL;
	async_count = 0;
}
